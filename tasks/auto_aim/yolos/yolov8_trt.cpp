#include "yolov8_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tasks/auto_aim/yolos/cuda_preprocess.hpp"

namespace auto_aim
{

// 自定义删除器
auto runtime_deleter_v8 = [](nvinfer1::IRuntime* p) { if(p) delete p; };
auto engine_deleter_v8 = [](nvinfer1::ICudaEngine* p) { if(p) delete p; };
auto context_deleter_v8 = [](nvinfer1::IExecutionContext* p) { if(p) delete p; };

YOLOV8_TRT::YOLOV8_TRT(const std::string & config_path, bool debug)
: debug_(debug),
  detector_(config_path, false),
  runtime_(nullptr, runtime_deleter_v8),
  engine_(nullptr, engine_deleter_v8),
  context_(nullptr, context_deleter_v8)
{
  auto yaml = YAML::LoadFile(config_path);

  // 读取配置（使用yolov8特定路径）
  engine_path_ = yaml["yolov8_engine_path"].as<std::string>("");
  onnx_path_ = yaml["yolov8_onnx_path"].as<std::string>("");
  device_ = yaml["device"].as<std::string>("GPU");
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  use_async_inference_ = yaml["use_async_inference"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();

  // ROI配置
  int x = yaml["roi"]["x"].as<int>();
  int y = yaml["roi"]["y"].as<int>();
  int width = yaml["roi"]["width"].as<int>();
  int height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  // 创建调试目录
  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  // 初始化CUDA
  cudaSetDevice(0);
  cudaStreamCreate(&stream_);

  // 初始化TensorRT
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("Failed to create TensorRT runtime");
  }

  // 加载或构建引擎
  if (!engine_path_.empty() && std::filesystem::exists(engine_path_)) {
    tools::logger()->info("[YOLOV8_TRT] Loading engine from: {}", engine_path_);
    loadEngine(engine_path_);
  } else if (!onnx_path_.empty() && std::filesystem::exists(onnx_path_)) {
    tools::logger()->info("[YOLOV8_TRT] Building engine from ONNX: {}", onnx_path_);
    buildEngineFromONNX(onnx_path_);
    if (!engine_path_.empty()) {
      serializeEngine(engine_path_);
    }
  } else {
    throw std::runtime_error(
      "No valid model found! Please provide either:\n"
      "  - yolov8_engine_path: path to .engine file\n"
      "  - yolov8_onnx_path: path to .onnx file");
  }

  // 创建执行上下文
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create execution context");
  }

  // 获取输入输出维度
  auto input_dims = engine_->getTensorShape(engine_->getIOTensorName(0));
  auto output_dims = engine_->getTensorShape(engine_->getIOTensorName(1));

  std::string input_name = engine_->getIOTensorName(0);
  std::string output_name = engine_->getIOTensorName(1);

  auto input_dtype = engine_->getTensorDataType(input_name.c_str());
  auto output_dtype = engine_->getTensorDataType(output_name.c_str());

  std::string input_dtype_str = (input_dtype == nvinfer1::DataType::kFLOAT) ? "FP32" :
                                 (input_dtype == nvinfer1::DataType::kHALF) ? "FP16" : "OTHER";
  std::string output_dtype_str = (output_dtype == nvinfer1::DataType::kFLOAT) ? "FP32" :
                                  (output_dtype == nvinfer1::DataType::kHALF) ? "FP16" : "OTHER";

  tools::logger()->warn("[YOLOV8_TRT] ===== Engine Tensor Info =====");
  tools::logger()->warn("[YOLOV8_TRT] Input: '{}' shape=[{},{},{},{}] dtype={}",
    input_name, input_dims.d[0], input_dims.d[1], input_dims.d[2], input_dims.d[3], input_dtype_str);
  tools::logger()->warn("[YOLOV8_TRT] Output: '{}' shape=[{},{},{}] dtype={}",
    output_name, output_dims.d[0], output_dims.d[1], output_dims.d[2], output_dtype_str);
  tools::logger()->warn("[YOLOV8_TRT] ==============================");

  // 修正：使用engine实际的输入尺寸
  input_w_ = input_dims.d[3];  // 从engine读取，应该是640
  input_h_ = input_dims.d[2];  // 从engine读取，应该是640

  // 修正：YOLOv8输出格式是[1, num_features, num_detections]
  // 需要转置：每个检测有num_features个值，共num_detections个检测
  output_data_size_ = output_dims.d[1];      // 每个检测的特征数 (28)
  output_num_detections_ = output_dims.d[2]; // 检测框数量 (8400)

  // 计算缓冲区大小
  input_size_ = 1 * 3 * input_h_ * input_w_ * sizeof(float);
  output_size_ = 1 * output_num_detections_ * output_data_size_ * sizeof(float);

  // 分配GPU内存
  cudaMalloc(&buffers_[0], input_size_);
  cudaMalloc(&buffers_[1], output_size_);

  // 使用Pinned Memory分配输入缓冲（优化H2D传输）
  cudaError_t err_input = cudaMallocHost((void**)&input_host_, input_size_);
  if (err_input != cudaSuccess) {
    tools::logger()->error("[YOLOV8_TRT] Failed to allocate input pinned memory: {}", cudaGetErrorString(err_input));
    throw std::runtime_error("Failed to allocate input pinned memory");
  } else {
    tools::logger()->info("[YOLOV8_TRT] Input using pinned memory for faster H2D transfer");
  }

  // 分配TopK GPU缓冲区（减少D2H传输）
  size_t topk_buffer_size = output_data_size_ * topk_max_ * sizeof(float);
  cudaError_t err_topk = cudaMalloc(&gpu_topk_buffer_, topk_buffer_size);
  if (err_topk != cudaSuccess) {
    tools::logger()->error("[YOLOV8_TRT] Failed to allocate GPU TopK buffer: {}", cudaGetErrorString(err_topk));
    cudaFreeHost(input_host_);
    throw std::runtime_error("Failed to allocate GPU TopK buffer");
  }

  // 使用Pinned Memory分配输出缓冲（优化D2H传输）
  // 只需为TopK数量分配空间（而不是全部8400）
  cudaError_t err_output = cudaMallocHost((void**)&output_host_, topk_max_ * output_data_size_ * sizeof(float));
  if (err_output != cudaSuccess) {
    tools::logger()->error("[YOLOV8_TRT] Failed to allocate output pinned memory: {}", cudaGetErrorString(err_output));
    cudaFreeHost(input_host_);
    cudaFree(gpu_topk_buffer_);
    throw std::runtime_error("Failed to allocate output pinned memory");
  } else {
    tools::logger()->info("[YOLOV8_TRT] Output using pinned memory for TopK={} (reduced D2H)", topk_max_);
  }

  // 预分配输入图像（使用普通内存，OpenCV操作）
  input_image_ = cv::Mat(input_h_, input_w_, CV_8UC3, cv::Scalar(0, 0, 0));

  // 为CUDA预处理分配GPU图像缓冲区 (支持最大1920x1200分辨率)
  gpu_img_buffer_size_ = 1920 * 1200 * 3 * sizeof(unsigned char);
  cudaError_t err_img = cudaMalloc(&gpu_img_buffer_, gpu_img_buffer_size_);
  if (err_img != cudaSuccess) {
    tools::logger()->error("[YOLOV8_TRT] Failed to allocate GPU image buffer: {}", cudaGetErrorString(err_img));
    cudaFreeHost(input_host_);
    cudaFreeHost(output_host_);
    cudaFree(gpu_topk_buffer_);
    throw std::runtime_error("Failed to allocate GPU image buffer");
  } else {
    tools::logger()->info("[YOLOV8_TRT] CUDA preprocessing enabled (GPU image buffer allocated)");
  }

  // 设置tensor地址
  context_->setTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
  context_->setTensorAddress(engine_->getIOTensorName(1), buffers_[1]);

  tools::logger()->info(
    "[YOLOV8_TRT] Initialized: {}x{} input, {} max detections, {} data per detection",
    input_w_, input_h_, output_num_detections_, output_data_size_);
}

YOLOV8_TRT::~YOLOV8_TRT()
{
  cudaFreeHost(input_host_);   // 释放输入pinned memory
  cudaFreeHost(output_host_);  // 释放输出pinned memory
  cudaFree(gpu_img_buffer_);   // 释放GPU图像缓冲区
  cudaFree(gpu_topk_buffer_);  // 释放GPU TopK缓冲区
  cudaFree(buffers_[0]);
  cudaFree(buffers_[1]);
  cudaStreamDestroy(stream_);
}

void YOLOV8_TRT::loadEngine(const std::string & engine_file)
{
  std::ifstream file(engine_file, std::ios::binary);
  if (!file.good()) {
    throw std::runtime_error("Failed to open engine file: " + engine_file);
  }

  file.seekg(0, std::ios::end);
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> engine_data(size);
  file.read(engine_data.data(), size);
  file.close();

  engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), size));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize engine");
  }

  tools::logger()->info("[YOLOV8_TRT] Engine loaded successfully");
}

void YOLOV8_TRT::buildEngineFromONNX(const std::string & onnx_file)
{
  auto builder = std::unique_ptr<nvinfer1::IBuilder, void(*)(nvinfer1::IBuilder*)>(
    nvinfer1::createInferBuilder(logger_), [](nvinfer1::IBuilder* p) { if(p) delete p; });

  auto network = std::unique_ptr<nvinfer1::INetworkDefinition, void(*)(nvinfer1::INetworkDefinition*)>(
    builder->createNetworkV2(0U), [](nvinfer1::INetworkDefinition* p) { if(p) delete p; });

  auto parser = std::unique_ptr<nvonnxparser::IParser, void(*)(nvonnxparser::IParser*)>(
    nvonnxparser::createParser(*network, logger_), [](nvonnxparser::IParser* p) { if(p) delete p; });

  if (!parser->parseFromFile(onnx_file.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    throw std::runtime_error("Failed to parse ONNX file");
  }

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig, void(*)(nvinfer1::IBuilderConfig*)>(
    builder->createBuilderConfig(), [](nvinfer1::IBuilderConfig* p) { if(p) delete p; });

  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1U << 30);

  auto serialized_engine = std::unique_ptr<nvinfer1::IHostMemory, void(*)(nvinfer1::IHostMemory*)>(
    builder->buildSerializedNetwork(*network, *config), [](nvinfer1::IHostMemory* p) { if(p) delete p; });

  if (!serialized_engine) {
    throw std::runtime_error("Failed to build engine");
  }

  engine_.reset(runtime_->deserializeCudaEngine(serialized_engine->data(), serialized_engine->size()));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize built engine");
  }

  tools::logger()->info("[YOLOV8_TRT] Engine built successfully");
}

void YOLOV8_TRT::serializeEngine(const std::string & engine_file)
{
  auto serialized = std::unique_ptr<nvinfer1::IHostMemory, void(*)(nvinfer1::IHostMemory*)>(
    engine_->serialize(), [](nvinfer1::IHostMemory* p) { if(p) delete p; });

  if (!serialized) {
    tools::logger()->error("[YOLOV8_TRT] Failed to serialize engine");
    return;
  }

  std::ofstream file(engine_file, std::ios::binary);
  if (!file.good()) {
    tools::logger()->error("[YOLOV8_TRT] Failed to open file for writing: {}", engine_file);
    return;
  }

  file.write(reinterpret_cast<const char *>(serialized->data()), serialized->size());
  file.close();

  tools::logger()->info("[YOLOV8_TRT] Engine serialized to: {}", engine_file);
}

std::list<Armor> YOLOV8_TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  // === 1. ROI提取 ===
  cv::Mat bgr_img;
  if (use_roi_) {
    bgr_img = raw_img(roi_).clone();
  } else {
    bgr_img = raw_img;
  }
  auto t_roi = std::chrono::high_resolution_clock::now();

  // === 2-3. CUDA预处理：Resize + Letterbox + Normalize + NCHW (全GPU完成) ===
  // 计算letterbox参数
  float scale = std::min(
    static_cast<float>(input_w_) / bgr_img.cols, static_cast<float>(input_h_) / bgr_img.rows);
  int pad_x = (input_w_ - static_cast<int>(bgr_img.cols * scale)) / 2;
  int pad_y = (input_h_ - static_cast<int>(bgr_img.rows * scale)) / 2;

  // 1) 将原始图像上传到GPU
  size_t img_size = bgr_img.cols * bgr_img.rows * 3 * sizeof(unsigned char);
  cudaMemcpyAsync(gpu_img_buffer_, bgr_img.data, img_size, cudaMemcpyHostToDevice, stream_);

  // 2) 调用CUDA kernel完成预处理（resize + letterbox + normalize + NCHW）
  cuda_preprocess_letterbox(
    gpu_img_buffer_,
    bgr_img.cols,
    bgr_img.rows,
    (float*)buffers_[0],  // 直接写入TensorRT输入buffer
    input_w_,
    input_h_,
    stream_);

  auto t_preprocess = std::chrono::high_resolution_clock::now();

  // === 4. 推理 ===
  if (use_async_inference_ && !first_frame_) {
    auto t_sync_start = std::chrono::high_resolution_clock::now();
    cudaStreamSynchronize(stream_);
    auto t_sync1 = std::chrono::high_resolution_clock::now();

    // GPU端TopK筛选，减少D2H传输
    int actual_topk = cuda_topk_filter(
      (float*)buffers_[1],
      output_data_size_,
      output_num_detections_,
      topk_max_,
      gpu_topk_buffer_,
      topk_threshold_,
      stream_);
    auto t_topk = std::chrono::high_resolution_clock::now();

    // 只传输TopK数据到CPU（大幅减少D2H）
    size_t topk_size = output_data_size_ * actual_topk * sizeof(float);
    cudaMemcpyAsync(output_host_, gpu_topk_buffer_, topk_size, cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    auto t_d2h = std::chrono::high_resolution_clock::now();

    // CUDA预处理已经完成，直接推理（无需H2D拷贝）
    context_->enqueueV3(stream_);
    auto t_infer = std::chrono::high_resolution_clock::now();

    // 直接使用channel-first layout数据，无需转置
    auto result = parse(prev_scale_, prev_pad_x_, prev_pad_y_, output_host_,
                       actual_topk, output_data_size_, prev_raw_img_, prev_frame_count_);
    auto t_parse = std::chrono::high_resolution_clock::now();

    // 打印计时（异步模式）
    tools::logger()->info(
      "[YOLOV8_TRT Async] Sync1: {:.2f}ms | TopK({}): {:.2f}ms | D2H({:.1f}KB): {:.2f}ms | Infer: {:.2f}ms | Parse: {:.2f}ms",
      std::chrono::duration<double, std::milli>(t_sync1 - t_sync_start).count(),
      actual_topk,
      std::chrono::duration<double, std::milli>(t_topk - t_sync1).count(),
      topk_size / 1024.0,
      std::chrono::duration<double, std::milli>(t_d2h - t_topk).count(),
      std::chrono::duration<double, std::milli>(t_infer - t_d2h).count(),
      std::chrono::duration<double, std::milli>(t_parse - t_infer).count()
    );

    prev_raw_img_ = bgr_img.clone();
    prev_scale_ = scale;
    prev_pad_x_ = pad_x;
    prev_pad_y_ = pad_y;
    prev_frame_count_ = frame_count;

    return result;
  } else {
    // CUDA预处理已经直接写入buffers_[0]，无需再H2D拷贝

    // 推理
    context_->enqueueV3(stream_);
    auto t_infer = std::chrono::high_resolution_clock::now();

    // GPU端TopK筛选，减少D2H传输
    int actual_topk = cuda_topk_filter(
      (float*)buffers_[1],
      output_data_size_,
      output_num_detections_,
      topk_max_,
      gpu_topk_buffer_,
      topk_threshold_,
      stream_);
    auto t_topk = std::chrono::high_resolution_clock::now();

    // 只传输TopK数据到CPU（大幅减少D2H）
    size_t topk_size = output_data_size_ * actual_topk * sizeof(float);
    cudaMemcpyAsync(output_host_, gpu_topk_buffer_, topk_size, cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);
    auto t_d2h = std::chrono::high_resolution_clock::now();

    // 直接使用channel-first layout数据，无需转置
    auto result = parse(scale, pad_x, pad_y, output_host_,
                       actual_topk, output_data_size_, bgr_img, frame_count);
    auto t_parse = std::chrono::high_resolution_clock::now();

    auto t_end = std::chrono::high_resolution_clock::now();

    // === 性能统计输出 ===
    double time_roi = std::chrono::duration<double, std::milli>(t_roi - t_start).count();
    double time_cuda_preprocess = std::chrono::duration<double, std::milli>(t_preprocess - t_roi).count();
    double time_infer = std::chrono::duration<double, std::milli>(t_infer - t_preprocess).count();
    double time_topk = std::chrono::duration<double, std::milli>(t_topk - t_infer).count();
    double time_d2h = std::chrono::duration<double, std::milli>(t_d2h - t_topk).count();
    double time_parse = std::chrono::duration<double, std::milli>(t_parse - t_d2h).count();
    double time_total = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    double preprocess_total = time_roi + time_cuda_preprocess;
    double inference_total = time_infer + time_topk + time_d2h;
    double postprocess_total = time_parse;

    tools::logger()->info(
      "[YOLOV8_TRT Performance] Frame {}: TOTAL={:.2f}ms | "
      "Preprocess={:.2f}ms (ROI:{:.2f} CUDA:{:.2f}) | "
      "Inference={:.2f}ms (Infer:{:.2f} TopK({}):{:.2f} D2H({:.1f}KB):{:.2f}) | "
      "Postprocess={:.2f}ms (Parse:{:.2f})",
      frame_count, time_total,
      preprocess_total, time_roi, time_cuda_preprocess,
      inference_total, time_infer, actual_topk, time_topk, topk_size / 1024.0, time_d2h,
      postprocess_total, time_parse
    );

    if (use_async_inference_) {
      first_frame_ = false;
      prev_raw_img_ = bgr_img.clone();
      prev_scale_ = scale;
      prev_pad_x_ = pad_x;
      prev_pad_y_ = pad_y;
      prev_frame_count_ = frame_count;
    }

    return result;
  }
}

std::list<Armor> YOLOV8_TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 注意：此函数保留为兼容接口，实际使用中应调用新的parse签名
  float* data = output.ptr<float>(0);
  return parse(scale, 0, 0, data, output.cols, output.rows, bgr_img, frame_count);
}

std::list<Armor> YOLOV8_TRT::parse(
  float scale, int pad_x, int pad_y, float* output_data, int num_detections, int data_size,
  const cv::Mat & bgr_img, int frame_count)
{
  auto t_parse_start = std::chrono::high_resolution_clock::now();

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> class_ids;
  std::vector<std::vector<cv::Point2f>> keypoints_list;

  const float inv_scale = 1.0f / scale;

  // YOLOv8-pose输出格式（channel-first layout）：
  // [0-3]: bbox (x, y, w, h)
  // [4-19]: 类别概率 (16个类：red_1到red_8，blue_1到blue_8)
  // [20-27]: 关键点坐标 (4个点×2坐标=8维)
  // 置信度 = max(类别概率)
  // 访问方式：output_data[channel * num_detections + detection_id]

  int debug_count = 0;
  float max_conf_all = 0.0f;

  for (int i = 0; i < num_detections; ++i) {
    // 找到16个类别中的最大概率
    float max_prob = 0.0f;
    int class_id = 0;
    for (int c = 0; c < 16; ++c) {
      float prob = output_data[(4 + c) * num_detections + i];
      if (prob > max_prob) {
        max_prob = prob;
        class_id = c;
      }
    }
    float conf = max_prob;

    max_conf_all = std::max(max_conf_all, conf);

    if (debug_count < 3 && conf > 0.01f) {
      tools::logger()->debug(
        "[YOLOV8_TRT] Detection {}: conf={:.3f}, class_id={} ({})",
        i, conf, class_id, class_id < 8 ? "red" : "blue");
      debug_count++;
    }

    if (conf < score_threshold_) continue;

    // 提取bbox（xywh格式转xyxy）
    float cx = output_data[0 * num_detections + i];
    float cy = output_data[1 * num_detections + i];
    float w = output_data[2 * num_detections + i];
    float h = output_data[3 * num_detections + i];

    float x1 = (cx - w / 2 - pad_x) * inv_scale;
    float y1 = (cy - h / 2 - pad_y) * inv_scale;
    float x2 = (cx + w / 2 - pad_x) * inv_scale;
    float y2 = (cy + h / 2 - pad_y) * inv_scale;

    cv::Rect box(
      static_cast<int>(x1),
      static_cast<int>(y1),
      static_cast<int>(x2 - x1),
      static_cast<int>(y2 - y1)
    );

    // 提取关键点（在channel[20-27]，4个点×2坐标）
    std::vector<cv::Point2f> kpts_raw;
    for (int k = 0; k < 4; ++k) {
      float kx = (output_data[(20 + k * 2) * num_detections + i] - pad_x) * inv_scale;
      float ky = (output_data[(20 + k * 2 + 1) * num_detections + i] - pad_y) * inv_scale;
      kpts_raw.emplace_back(kx, ky);
    }

    // 模型输出顺序：[TR, BR, BL, TL] (kpt0, kpt1, kpt2, kpt3)
    // 代码期望顺序：[TL, TR, BR, BL]
    // 重排：[kpt3, kpt0, kpt1, kpt2] -> [TL, TR, BR, BL]
    std::vector<cv::Point2f> kpts = {kpts_raw[3], kpts_raw[0], kpts_raw[1], kpts_raw[2]};

    // 直接使用原始class_id（0-15），由yolov8_trt_armor_properties映射表处理
    boxes.push_back(box);
    confidences.push_back(conf);
    class_ids.push_back(class_id);  // 保留原始class_id
    keypoints_list.push_back(kpts);
  }

  auto t_detection_loop = std::chrono::high_resolution_clock::now();

  tools::logger()->info(
    "[YOLOV8_TRT] Frame {}: found {} detections above threshold, max_conf={:.3f}",
    frame_count, boxes.size(), max_conf_all);

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);
  auto t_nms = std::chrono::high_resolution_clock::now();

  std::list<Armor> armors;
  for (int idx : indices) {
    auto & kpts = keypoints_list[idx];

    if (use_roi_) {
      for (auto & pt : kpts) {
        pt += offset_;
      }
      boxes[idx].x += static_cast<int>(offset_.x);
      boxes[idx].y += static_cast<int>(offset_.y);
    }

    try {
      // YOLOV8_TRT使用专用的类别映射表
      Armor armor(
        class_ids[idx],
        confidences[idx],
        boxes[idx],
        kpts,
        YOLOVersion::YOLOV8_TRT
      );

      // 调试：打印Armor信息
      tools::logger()->debug(
        "[YOLOV8_TRT] Armor created: class_id={}, conf={:.3f}, type={}, color={}, name={}, ratio={:.2f}",
        class_ids[idx],
        armor.confidence,
        armor.type == ArmorType::small ? "small" : "big",
        armor.color == Color::red ? "red" : (armor.color == Color::blue ? "blue" : "other"),
        static_cast<int>(armor.name),
        armor.ratio);

      bool name_ok = check_name(armor);
      bool type_ok = check_type(armor);

      tools::logger()->debug(
        "[YOLOV8_TRT] Armor checks: name_ok={}, type_ok={}, min_conf={:.3f}",
        name_ok, type_ok, min_confidence_);

      if (!name_ok || !type_ok) {
        continue;
      }

      armors.push_back(armor);
    } catch (const std::exception& e) {
      tools::logger()->warn("[YOLOV8_TRT] Failed to create armor: {}", e.what());
      continue;
    }
  }

  auto t_armor_creation = std::chrono::high_resolution_clock::now();

  if (debug_) {
    draw_detections(bgr_img, armors, frame_count);
  }
  auto t_debug = std::chrono::high_resolution_clock::now();

  // === Parse函数内部计时 ===
  double time_detection = std::chrono::duration<double, std::milli>(t_detection_loop - t_parse_start).count();
  double time_nms = std::chrono::duration<double, std::milli>(t_nms - t_detection_loop).count();
  double time_armor = std::chrono::duration<double, std::milli>(t_armor_creation - t_nms).count();
  double time_debug_draw = std::chrono::duration<double, std::milli>(t_debug - t_armor_creation).count();

  tools::logger()->debug(
    "[YOLOV8_TRT Parse] Detection_loop={:.2f}ms | NMS={:.2f}ms | Armor_creation={:.2f}ms | Debug_draw={:.2f}ms",
    time_detection, time_nms, time_armor, time_debug_draw
  );

  return armors;
}

bool YOLOV8_TRT::check_name(const Armor & armor) const
{
  return armor.confidence >= min_confidence_;
}

bool YOLOV8_TRT::check_type(const Armor & armor) const
{
  if (armor.points.size() != 4) {
    tools::logger()->debug("[YOLOV8_TRT] check_type failed: points.size()={}", armor.points.size());
    return false;
  }

  float width = cv::norm(armor.points[1] - armor.points[0]);
  float height = cv::norm(armor.points[2] - armor.points[1]);
  float ratio = width / std::max(height, 1.0f);

  bool ok = ratio > 0.5f && ratio < 5.0f;
  if (!ok) {
    tools::logger()->debug(
      "[YOLOV8_TRT] check_type failed: width={:.1f}, height={:.1f}, ratio={:.2f} (need 0.5-5.0)",
      width, height, ratio);
  }

  return ok;
}

void YOLOV8_TRT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  cv::Mat vis = img.clone();
  for (const auto & armor : armors) {
    for (size_t i = 0; i < armor.points.size(); ++i) {
      cv::circle(vis, armor.points[i], 3, cv::Scalar(0, 255, 0), -1);
      cv::line(vis, armor.points[i], armor.points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
    }
    cv::putText(
      vis, fmt::format("{:.2f}", armor.confidence), armor.points[0],
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
  }
  cv::imwrite(fmt::format("{}/frame_{:04d}.jpg", save_path_, frame_count), vis);
}

}  // namespace auto_aim
