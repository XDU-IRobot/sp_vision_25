#include "cuda_preprocess.hpp"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <algorithm>

namespace auto_aim {

/**
 * CUDA kernel: 同时完成 resize + letterbox + normalize + BGR2NCHW
 *
 * 线程组织：每个线程处理输出图像的一个像素
 */
__global__ void preprocess_kernel(
    const unsigned char* src,
    int src_width,
    int src_height,
    float* dst,
    int dst_width,
    int dst_height,
    float scale,
    int pad_x,
    int pad_y)
{
    int dst_x = blockIdx.x * blockDim.x + threadIdx.x;
    int dst_y = blockIdx.y * blockDim.y + threadIdx.y;

    if (dst_x >= dst_width || dst_y >= dst_height) return;

    // 计算在letterbox中的位置
    int resized_x = dst_x - pad_x;
    int resized_y = dst_y - pad_y;

    int resized_width = static_cast<int>(src_width * scale);
    int resized_height = static_cast<int>(src_height * scale);

    // 如果在padding区域，填充黑色 (0, 0, 0)
    if (resized_x < 0 || resized_x >= resized_width ||
        resized_y < 0 || resized_y >= resized_height) {
        int hw = dst_width * dst_height;
        int idx = dst_y * dst_width + dst_x;
        dst[0 * hw + idx] = 0.0f;  // B
        dst[1 * hw + idx] = 0.0f;  // G
        dst[2 * hw + idx] = 0.0f;  // R
        return;
    }

    // 映射回原图坐标 (使用最近邻插值)
    float src_x_f = resized_x / scale;
    float src_y_f = resized_y / scale;

    int src_x = static_cast<int>(src_x_f);
    int src_y = static_cast<int>(src_y_f);

    // 边界检查
    src_x = min(max(src_x, 0), src_width - 1);
    src_y = min(max(src_y, 0), src_height - 1);

    // 从HWC格式读取BGR像素
    int src_idx = (src_y * src_width + src_x) * 3;
    unsigned char b = src[src_idx + 0];
    unsigned char g = src[src_idx + 1];
    unsigned char r = src[src_idx + 2];

    // 归一化并写入NCHW格式 (保持BGR顺序)
    int hw = dst_width * dst_height;
    int dst_idx = dst_y * dst_width + dst_x;
    dst[0 * hw + dst_idx] = b / 255.0f;  // B通道
    dst[1 * hw + dst_idx] = g / 255.0f;  // G通道
    dst[2 * hw + dst_idx] = r / 255.0f;  // R通道
}

void cuda_preprocess_letterbox(
    const unsigned char* src_device,
    int src_width,
    int src_height,
    float* dst_device,
    int dst_width,
    int dst_height,
    cudaStream_t stream)
{
    // 计算letterbox参数
    float scale = fminf(
        static_cast<float>(dst_width) / src_width,
        static_cast<float>(dst_height) / src_height);

    int resized_width = static_cast<int>(src_width * scale);
    int resized_height = static_cast<int>(src_height * scale);
    int pad_x = (dst_width - resized_width) / 2;
    int pad_y = (dst_height - resized_height) / 2;

    // 配置线程块和网格
    dim3 block(16, 16);  // 16x16 = 256个线程
    dim3 grid(
        (dst_width + block.x - 1) / block.x,
        (dst_height + block.y - 1) / block.y);

    // 启动kernel
    preprocess_kernel<<<grid, block, 0, stream>>>(
        src_device,
        src_width,
        src_height,
        dst_device,
        dst_width,
        dst_height,
        scale,
        pad_x,
        pad_y);
}

/**
 * CUDA kernel: 计算每个detection的置信度
 * 置信度 = max(class_prob[0:16])，即16个类别概率的最大值
 */
__global__ void compute_confidence_kernel(
    const float* input,
    int num_features,
    int num_detections,
    float* confidences)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_detections) return;

    // YOLOv8格式：channel [4-19] 是16个类别概率
    float max_prob = 0.0f;
    for (int c = 4; c < 20; ++c) {
        float prob = input[c * num_detections + idx];
        max_prob = fmaxf(max_prob, prob);
    }
    confidences[idx] = max_prob;
}

/**
 * CUDA kernel: 根据TopK索引提取特征数据
 */
__global__ void extract_topk_kernel(
    const float* input,
    const int* topk_indices,
    int num_features,
    int num_detections,
    int topk,
    float* output)
{
    int feature_idx = blockIdx.y;
    int k_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (k_idx >= topk || feature_idx >= num_features) return;

    int detection_idx = topk_indices[k_idx];
    output[feature_idx * topk + k_idx] = input[feature_idx * num_detections + detection_idx];
}

/**
 * CUDA kernel: 筛选高置信度检测并compact输出
 */
__global__ void filter_and_compact_kernel(
    const float* input,
    int num_features,
    int num_detections,
    float* confidences,
    int* indices,
    int* valid_count,
    float score_threshold)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_detections) return;

    // 计算置信度
    float max_prob = 0.0f;
    for (int c = 4; c < 20; ++c) {
        float prob = input[c * num_detections + idx];
        max_prob = fmaxf(max_prob, prob);
    }

    // 如果超过阈值，添加到输出
    if (max_prob >= score_threshold) {
        int out_idx = atomicAdd(valid_count, 1);
        confidences[out_idx] = max_prob;
        indices[out_idx] = idx;
    }
}

/**
 * TopK筛选实现（优化版：先过滤+静态预分配+无GPU→CPU同步）
 */
int cuda_topk_filter(
    const float* input_device,
    int num_features,
    int num_detections,
    int topk,
    float* output_device,
    float score_threshold,
    cudaStream_t stream)
{
    // 静态预分配缓冲区（避免每帧分配）
    static thrust::device_vector<float> confidences;
    static thrust::device_vector<int> indices;
    static thrust::device_vector<int> valid_count_dev(1);
    static int last_num_detections = 0;

    // 只在尺寸变化时重新分配
    if (last_num_detections != num_detections) {
        confidences.resize(num_detections);
        indices.resize(num_detections);
        last_num_detections = num_detections;
    }

    // 1. 重置计数器
    cudaMemsetAsync(thrust::raw_pointer_cast(valid_count_dev.data()), 0, sizeof(int), stream);

    // 2. 筛选高置信度检测（GPU并行）
    dim3 block(256);
    dim3 grid((num_detections + block.x - 1) / block.x);
    filter_and_compact_kernel<<<grid, block, 0, stream>>>(
        input_device,
        num_features,
        num_detections,
        thrust::raw_pointer_cast(confidences.data()),
        thrust::raw_pointer_cast(indices.data()),
        thrust::raw_pointer_cast(valid_count_dev.data()),
        score_threshold);

    // 3. 获取有效数量（需要同步，但只拷贝1个int）
    int valid_count = 0;
    cudaMemcpyAsync(&valid_count, thrust::raw_pointer_cast(valid_count_dev.data()),
                    sizeof(int), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // 4. 如果有效检测少于topk，直接使用；否则需要排序TopK
    int actual_topk = (valid_count <= topk) ? valid_count : topk;

    if (actual_topk == 0) {
        return 0;
    }

    // 5. 如果有效数量大于topk，对前valid_count个排序并取TopK
    if (valid_count > topk) {
        thrust::sort_by_key(
            thrust::cuda::par.on(stream),
            confidences.begin(),
            confidences.begin() + valid_count,
            indices.begin(),
            thrust::greater<float>());
    }

    // 6. 根据TopK索引提取特征数据
    dim3 extract_block(256);
    dim3 extract_grid(
        (actual_topk + extract_block.x - 1) / extract_block.x,
        num_features);

    extract_topk_kernel<<<extract_grid, extract_block, 0, stream>>>(
        input_device,
        thrust::raw_pointer_cast(indices.data()),
        num_features,
        num_detections,
        actual_topk,
        output_device);

    return actual_topk;
}

}  // namespace auto_aim
