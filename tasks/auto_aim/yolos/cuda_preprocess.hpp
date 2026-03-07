#ifndef AUTO_AIM__CUDA_PREPROCESS_HPP
#define AUTO_AIM__CUDA_PREPROCESS_HPP

#include <cuda_runtime.h>

namespace auto_aim {

/**
 * CUDA预处理kernel：完成resize + letterbox + normalize + BGR2NCHW
 *
 * 输入：GPU上的原始图像 (HWC格式, BGR, uint8)
 * 输出：GPU上的预处理后数据 (NCHW格式, BGR, float32, 归一化到[0,1])
 *
 * @param src_device GPU上的原始图像数据 (HWC格式, BGR, uint8)
 * @param src_width 原始图像宽度
 * @param src_height 原始图像高度
 * @param dst_device GPU上的目标buffer (NCHW格式, BGR, float32, 已归一化)
 * @param dst_width 目标宽度 (如640)
 * @param dst_height 目标高度 (如640)
 * @param stream CUDA流
 */
void cuda_preprocess_letterbox(
    const unsigned char* src_device,
    int src_width,
    int src_height,
    float* dst_device,
    int dst_width,
    int dst_height,
    cudaStream_t stream);

/**
 * CUDA后处理：TopK筛选，减少D2H传输量
 *
 * 输入：TensorRT输出 [num_features, num_detections] (channel-first)
 * 输出：TopK检测结果 [num_features, topk]
 *
 * @param input_device GPU上的TensorRT输出 [num_features, num_detections]
 * @param num_features 特征数量 (28)
 * @param num_detections 检测数量 (8400)
 * @param topk 保留的top检测数量 (例如1000)
 * @param output_device GPU上的输出buffer [num_features, topk]
 * @param score_threshold 置信度阈值（低于此值的不参与TopK）
 * @param stream CUDA流
 * @return 实际筛选出的检测数量（<=topk）
 */
int cuda_topk_filter(
    const float* input_device,
    int num_features,
    int num_detections,
    int topk,
    float* output_device,
    float score_threshold,
    cudaStream_t stream);

}  // namespace auto_aim

#endif  // AUTO_AIM__CUDA_PREPROCESS_HPP
