#include "yolo26_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "cuda_preprocess.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/path.hpp"

namespace auto_aim
{

namespace
{
// TensorRT 对象需要手动 delete，这里用自定义 deleter 交给 unique_ptr 托管。
auto runtime_deleter_v26 = [](nvinfer1::IRuntime * p) { if (p) delete p; };
auto engine_deleter_v26 = [](nvinfer1::ICudaEngine * p) { if (p) delete p; };
auto context_deleter_v26 = [](nvinfer1::IExecutionContext * p) { if (p) delete p; };

void cuda_check(cudaError_t code, const std::string & where)
{
  if (code != cudaSuccess) {
    throw std::runtime_error(
            "CUDA failure at " + where + ": " + cudaGetErrorString(code) +
            " (" + std::to_string(static_cast<int>(code)) + ")");
  }
}

void cuda_preflight(int device_id)
{
  int device_count = 0;
  cuda_check(cudaGetDeviceCount(&device_count), "cudaGetDeviceCount");
  if (device_count <= 0) {
    throw std::runtime_error("No CUDA device available");
  }
  if (device_id < 0 || device_id >= device_count) {
    throw std::runtime_error("Invalid CUDA device id: " + std::to_string(device_id));
  }
  cuda_check(cudaSetDevice(device_id), "cudaSetDevice");
  cuda_check(cudaFree(nullptr), "cudaFree(nullptr) preflight");
}

// 根据 TensorRT 的数据类型返回单个元素字节数，用于计算显存/内存大小。
size_t get_dtype_size(nvinfer1::DataType dtype)
{
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return sizeof(float);
    case nvinfer1::DataType::kHALF:
      return sizeof(__half);
    default:
      return 0;
  }
}
}  // namespace

YOLO26_TRT::YOLO26_TRT(const std::string & config_path, bool debug)
: debug_(debug),
  runtime_(nullptr, runtime_deleter_v26),
  engine_(nullptr, engine_deleter_v26),
  context_(nullptr, context_deleter_v26),
  detector_(config_path, false)
{
  // 读取配置并初始化路径、阈值、ROI、CUDA 资源、TensorRT 引擎。
  auto yaml = YAML::LoadFile(config_path);

  {
    const auto raw_engine_path = yaml["yolo26_engine_path"].as<std::string>("");
    if (!raw_engine_path.empty()) {
      std::filesystem::path p(raw_engine_path);
      engine_path_ = p.is_absolute() ? p.string() : p.lexically_normal().string();
    }
  }
  onnx_path_ = tools::resolve_path_from_config(
    config_path, yaml["yolo26_onnx_path"].as<std::string>(""));
  const bool engine_exists = !engine_path_.empty() && std::filesystem::exists(engine_path_);
  const bool onnx_exists = !onnx_path_.empty() && std::filesystem::exists(onnx_path_);
  tools::logger()->info(
    "[YOLO26_TRT] model paths: engine='{}' (exists={}), onnx='{}' (exists={})",
    engine_path_, engine_exists, onnx_path_, onnx_exists);
  device_ = yaml["device"].as<std::string>("GPU");
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  score_threshold_ = yaml["yolo26_score_threshold"].as<float>(0.2f);
  nms_threshold_ = yaml["yolo26_nms_threshold"].as<float>(0.3f);
  use_nms_ = yaml["yolo26_use_nms"].as<bool>(true);
  swap_rb_channels_ = yaml["yolo26_trt_swap_rb"].as<bool>(false);
  use_cuda_preprocess_ = yaml["yolo26_use_cuda_preprocess"].as<bool>(false);
  tools::logger()->info(
    "[YOLO26_TRT] thresholds: score_th={:.3f}, nms_th={:.3f}, min_conf={:.3f}",
    score_threshold_, nms_threshold_, min_confidence_);
  tools::logger()->info("[YOLO26_TRT] preprocess: swap_rb_channels={}", swap_rb_channels_);
  tools::logger()->info("[YOLO26_TRT] preprocess: use_cuda_preprocess={}", use_cuda_preprocess_);
  tools::logger()->info("[YOLO26_TRT] postprocess: use_nms={}", use_nms_);

  int x = yaml["roi"]["x"].as<int>();
  int y = yaml["roi"]["y"].as<int>();
  int width = yaml["roi"]["width"].as<int>();
  int height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);

  cuda_preflight(0);
  cuda_check(cudaStreamCreate(&stream_), "cudaStreamCreate");

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("Failed to create TensorRT runtime");
  }

  if (engine_exists) {
    // 优先直接加载已经序列化好的 TensorRT engine，启动更快。
    tools::logger()->info("[YOLO26_TRT] Loading engine from: {}", engine_path_);
    loadEngine(engine_path_);
  } else if (onnx_exists) {
    // 没有 engine 时，从 ONNX 现场构建 engine，并可选落盘缓存。
    tools::logger()->info("[YOLO26_TRT] Building engine from ONNX: {}", onnx_path_);
    buildEngineFromONNX(onnx_path_);
    if (!engine_path_.empty()) {
      serializeEngine(engine_path_);
    }
  } else {
    tools::logger()->error(
      "[YOLO26_TRT] Invalid model paths. engine='{}', onnx='{}'",
      engine_path_, onnx_path_);
    throw std::runtime_error(
      "No valid model found! Please provide either:\n"
      "  - yolo26_engine_path: path to .engine file\n"
      "  - yolo26_onnx_path: path to .onnx file");
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create execution context");
  }

  const char * input_name = engine_->getIOTensorName(0);
  const char * output_name = engine_->getIOTensorName(1);
  auto input_dims = engine_->getTensorShape(input_name);

  input_dtype_ = engine_->getTensorDataType(input_name);
  output_dtype_ = engine_->getTensorDataType(output_name);

  if (input_dims.nbDims == 4 && input_dims.d[2] > 0 && input_dims.d[3] > 0) {
    input_h_ = input_dims.d[2];
    input_w_ = input_dims.d[3];
  }

  if (input_dims.nbDims == 4 && (input_dims.d[0] <= 0 || input_dims.d[1] <= 0 || input_dims.d[2] <= 0 || input_dims.d[3] <= 0)) {
    // 动态 shape 模型：在 context 上固定一次推理输入尺寸。
    nvinfer1::Dims4 fixed_input_shape{1, 3, input_h_, input_w_};
    if (!context_->setInputShape(input_name, fixed_input_shape)) {
      throw std::runtime_error("Failed to set dynamic input shape for YOLO26_TRT");
    }
  }

  auto runtime_input_dims = context_->getTensorShape(input_name);
  auto runtime_output_dims = context_->getTensorShape(output_name);

  if (runtime_input_dims.nbDims == 4 && runtime_input_dims.d[2] > 0 && runtime_input_dims.d[3] > 0) {
    input_h_ = runtime_input_dims.d[2];
    input_w_ = runtime_input_dims.d[3];
  }

  const size_t input_elements = static_cast<size_t>(1) * 3 * input_h_ * input_w_;
  input_size_bytes_ = input_elements * get_dtype_size(input_dtype_);

  size_t output_elements = 1;
  for (int i = 0; i < runtime_output_dims.nbDims; ++i) {
    output_elements *= static_cast<size_t>(runtime_output_dims.d[i] > 0 ? runtime_output_dims.d[i] : 1);
  }
  output_size_bytes_ = output_elements * get_dtype_size(output_dtype_);

  // 自动解析输出布局：支持 [N,rows,stride] / [N,stride,rows] 等形状。
  prepareOutputLayoutFromEngine(output_name);

  if (input_dtype_ == nvinfer1::DataType::kFLOAT) {
    input_host_float_.resize(input_elements);
  } else if (input_dtype_ == nvinfer1::DataType::kHALF) {
    input_host_half_.resize(input_elements);
  } else {
    throw std::runtime_error("YOLO26_TRT only supports FP32/FP16 input tensors");
  }

  if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
    output_host_float_.resize(output_elements);
  } else if (output_dtype_ == nvinfer1::DataType::kHALF) {
    output_host_half_.resize(output_elements);
    output_host_float_.resize(output_elements);
  } else {
    throw std::runtime_error("YOLO26_TRT only supports FP32/FP16 output tensors");
  }

  cuda_check(
    cudaMalloc(reinterpret_cast<void **>(&buffers_[0]), input_size_bytes_),
    "cudaMalloc(input)");
  cuda_check(
    cudaMalloc(reinterpret_cast<void **>(&buffers_[1]), output_size_bytes_),
    "cudaMalloc(output)");

  context_->setTensorAddress(input_name, buffers_[0]);
  context_->setTensorAddress(output_name, buffers_[1]);

  input_image_ = cv::Mat(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));

  tools::logger()->info(
    "[YOLO26_TRT] Initialized: {}x{} input, output rows={}, stride={}, channel_first={}",
    input_w_, input_h_, output_rows_, output_stride_, output_channel_first_);
}

YOLO26_TRT::~YOLO26_TRT()
{
  // 释放显存与 CUDA stream。
  if (gpu_img_buffer_) cudaFree(gpu_img_buffer_);
  if (buffers_[0]) cudaFree(buffers_[0]);
  if (buffers_[1]) cudaFree(buffers_[1]);
  if (stream_) cudaStreamDestroy(stream_);
}

void YOLO26_TRT::loadEngine(const std::string & engine_file)
{
  // 从磁盘读取 engine 二进制并反序列化为 ICudaEngine。
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
}

void YOLO26_TRT::buildEngineFromONNX(const std::string & onnx_file)
{
  // TensorRT build 流程：Builder -> Network -> Parser -> Config -> SerializedEngine。
  auto builder = std::unique_ptr<nvinfer1::IBuilder, void(*)(nvinfer1::IBuilder*)>(
    nvinfer1::createInferBuilder(logger_), [](nvinfer1::IBuilder * p) { if (p) delete p; });

  const auto flags = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition, void(*)(nvinfer1::INetworkDefinition*)>(
    builder->createNetworkV2(flags), [](nvinfer1::INetworkDefinition * p) { if (p) delete p; });

  auto parser = std::unique_ptr<nvonnxparser::IParser, void(*)(nvonnxparser::IParser*)>(
    nvonnxparser::createParser(*network, logger_), [](nvonnxparser::IParser * p) { if (p) delete p; });

  if (!parser->parseFromFile(onnx_file.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    throw std::runtime_error("Failed to parse ONNX file: " + onnx_file);
  }

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig, void(*)(nvinfer1::IBuilderConfig*)>(
    builder->createBuilderConfig(), [](nvinfer1::IBuilderConfig * p) { if (p) delete p; });

  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);

  auto input_tensor = network->getInput(0);
  if (!input_tensor) {
    throw std::runtime_error("Failed to get ONNX input tensor for TensorRT build");
  }

  auto input_dims = input_tensor->getDimensions();
  bool has_dynamic_input = false;
  for (int i = 0; i < input_dims.nbDims; ++i) {
    if (input_dims.d[i] <= 0) {
      has_dynamic_input = true;
      break;
    }
  }

  if (has_dynamic_input) {
    // 若 ONNX 输入含动态维度，为其创建固定范围的 optimization profile。
    auto profile = builder->createOptimizationProfile();
    if (!profile) {
      throw std::runtime_error("Failed to create TensorRT optimization profile");
    }

    nvinfer1::Dims min_dims = input_dims;
    nvinfer1::Dims opt_dims = input_dims;
    nvinfer1::Dims max_dims = input_dims;

    for (int i = 0; i < input_dims.nbDims; ++i) {
      if (input_dims.d[i] > 0) {
        min_dims.d[i] = input_dims.d[i];
        opt_dims.d[i] = input_dims.d[i];
        max_dims.d[i] = input_dims.d[i];
        continue;
      }

      if (i == 0) {
        min_dims.d[i] = 1;
        opt_dims.d[i] = 1;
        max_dims.d[i] = 1;
      } else if (i == 1) {
        min_dims.d[i] = 3;
        opt_dims.d[i] = 3;
        max_dims.d[i] = 3;
      } else if (i == 2) {
        min_dims.d[i] = input_h_;
        opt_dims.d[i] = input_h_;
        max_dims.d[i] = input_h_;
      } else if (i == 3) {
        min_dims.d[i] = input_w_;
        opt_dims.d[i] = input_w_;
        max_dims.d[i] = input_w_;
      } else {
        min_dims.d[i] = 1;
        opt_dims.d[i] = 1;
        max_dims.d[i] = 1;
      }
    }

    const char * input_name = input_tensor->getName();
    const bool ok_min = profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMIN, min_dims);
    const bool ok_opt = profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kOPT, opt_dims);
    const bool ok_max = profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMAX, max_dims);
    if (!ok_min || !ok_opt || !ok_max) {
      throw std::runtime_error("Failed to set TensorRT optimization profile dimensions");
    }

    config->addOptimizationProfile(profile);
    tools::logger()->info("[YOLO26_TRT] Added optimization profile for dynamic ONNX input");
  }

  auto serialized_engine = std::unique_ptr<nvinfer1::IHostMemory, void(*)(nvinfer1::IHostMemory*)>(
    builder->buildSerializedNetwork(*network, *config), [](nvinfer1::IHostMemory * p) { if (p) delete p; });

  if (!serialized_engine) {
    throw std::runtime_error("Failed to build TensorRT engine from ONNX");
  }

  engine_.reset(runtime_->deserializeCudaEngine(serialized_engine->data(), serialized_engine->size()));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize built engine");
  }
}

void YOLO26_TRT::serializeEngine(const std::string & engine_file)
{
  // 将构建好的 engine 序列化到磁盘，减少后续冷启动耗时。
  auto serialized = std::unique_ptr<nvinfer1::IHostMemory, void(*)(nvinfer1::IHostMemory*)>(
    engine_->serialize(), [](nvinfer1::IHostMemory * p) { if (p) delete p; });

  if (!serialized) {
    tools::logger()->warn("[YOLO26_TRT] Failed to serialize engine");
    return;
  }

  std::filesystem::create_directories(std::filesystem::path(engine_file).parent_path());
  std::ofstream file(engine_file, std::ios::binary);
  if (!file.good()) {
    tools::logger()->warn("[YOLO26_TRT] Failed to open {} for engine serialization", engine_file);
    return;
  }

  file.write(reinterpret_cast<const char *>(serialized->data()), serialized->size());
}

void YOLO26_TRT::prepareOutputLayoutFromEngine(const char * output_name)
{
  // 根据输出 tensor 维度推断“候选数 rows”和“每个候选步长 stride”。
  // stride 支持 14/18/28：分别对应不同导出格式（bbox+cls+kpts）。
  auto output_dims = context_->getTensorShape(output_name);

  output_rows_ = 0;
  output_stride_ = 14;
  output_channel_first_ = false;

  if (output_dims.nbDims == 3) {
    if (output_dims.d[2] == 14 || output_dims.d[2] == 18 || output_dims.d[2] == 28) {
      output_rows_ = output_dims.d[1];
      output_stride_ = output_dims.d[2];
      output_channel_first_ = false;
      return;
    }
    if (output_dims.d[1] == 14 || output_dims.d[1] == 18 || output_dims.d[1] == 28) {
      output_rows_ = output_dims.d[2];
      output_stride_ = output_dims.d[1];
      output_channel_first_ = true;
      return;
    }
  }

  if (output_dims.nbDims == 2) {
    if (output_dims.d[1] == 14 || output_dims.d[1] == 18 || output_dims.d[1] == 28) {
      output_rows_ = output_dims.d[0];
      output_stride_ = output_dims.d[1];
      output_channel_first_ = false;
      return;
    }
    if (output_dims.d[0] == 14 || output_dims.d[0] == 18 || output_dims.d[0] == 28) {
      output_rows_ = output_dims.d[1];
      output_stride_ = output_dims.d[0];
      output_channel_first_ = true;
      return;
    }
  }

  throw std::runtime_error("YOLO26_TRT unsupported output shape");
}

std::list<Armor> YOLO26_TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  // 主推理入口：预处理 -> H2D -> TensorRT 推理 -> D2H -> 解析后处理。
  if (raw_img.empty()) {
    return {};
  }

  cv::Mat bgr_img;
  tmp_img_ = raw_img;
  if (use_roi_) {
    // 开启 ROI 时仅在局部区域检测，后续通过 offset_ 还原回全图坐标。
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  const int orig_w = bgr_img.cols;
  const int orig_h = bgr_img.rows;
  const float scale = std::min(static_cast<float>(input_w_) / orig_w, static_cast<float>(input_h_) / orig_h);
  const int new_w = static_cast<int>(orig_w * scale);
  const int new_h = static_cast<int>(orig_h * scale);
  const int pad_x = (input_w_ - new_w) / 2;
  const int pad_y = (input_h_ - new_h) / 2;

  // 每帧先重置整张输入图，避免上一帧残留污染当前帧 padding 区域。
  input_image_.setTo(cv::Scalar(114, 114, 114));

  const bool use_cuda_path = use_cuda_preprocess_ && input_dtype_ == nvinfer1::DataType::kFLOAT;
  if (use_cuda_path) {
    const cv::Mat src_contiguous = bgr_img.isContinuous() ? bgr_img : bgr_img.clone();
    const size_t img_size =
      static_cast<size_t>(src_contiguous.cols) * src_contiguous.rows * 3 * sizeof(unsigned char);
    if (img_size > gpu_img_buffer_bytes_) {
      if (gpu_img_buffer_) {
        cudaFree(gpu_img_buffer_);
        gpu_img_buffer_ = nullptr;
      }
      cuda_check(
        cudaMalloc(reinterpret_cast<void **>(&gpu_img_buffer_), img_size),
        "cudaMalloc(gpu_img_buffer)");
      gpu_img_buffer_bytes_ = img_size;
    }

    cuda_check(
      cudaMemcpyAsync(gpu_img_buffer_, src_contiguous.data, img_size, cudaMemcpyHostToDevice, stream_),
      "cudaMemcpyAsync(H2D,raw_image)");
    cuda_preprocess_letterbox(
      gpu_img_buffer_, src_contiguous.cols, src_contiguous.rows,
      reinterpret_cast<float *>(buffers_[0]), input_w_, input_h_, stream_);
  } else {
    // Letterbox：按比例缩放并居中填充到网络输入尺寸。
    cv::resize(
      bgr_img,
      input_image_(cv::Rect(pad_x, pad_y, new_w, new_h)),
      cv::Size(new_w, new_h),
      0, 0,
      cv::INTER_LINEAR);
  }

  const size_t plane_size = static_cast<size_t>(input_h_) * input_w_;
  const int channel_map[3] = {
    swap_rb_channels_ ? 2 : 0,
    1,
    swap_rb_channels_ ? 0 : 2
  };
  // 按 NCHW 把图像转换到输入缓存，并归一化到 [0,1]。
  if (use_cuda_path) {
    // CUDA 路径已经把预处理结果直接写入 TensorRT 输入 buffer。
  } else if (input_dtype_ == nvinfer1::DataType::kFLOAT) {
    for (int c = 0; c < 3; ++c) {
      for (int h = 0; h < input_h_; ++h) {
        for (int w = 0; w < input_w_; ++w) {
          input_host_float_[c * plane_size + h * input_w_ + w] =
            input_image_.at<cv::Vec3b>(h, w)[channel_map[c]] / 255.0f;
        }
      }
    }
    cuda_check(
      cudaMemcpyAsync(buffers_[0], input_host_float_.data(), input_size_bytes_, cudaMemcpyHostToDevice, stream_),
      "cudaMemcpyAsync(H2D,input,float)");
  } else {
    for (int c = 0; c < 3; ++c) {
      for (int h = 0; h < input_h_; ++h) {
        for (int w = 0; w < input_w_; ++w) {
          input_host_half_[c * plane_size + h * input_w_ + w] =
            __float2half(input_image_.at<cv::Vec3b>(h, w)[channel_map[c]] / 255.0f);
        }
      }
    }
    cuda_check(
      cudaMemcpyAsync(buffers_[0], input_host_half_.data(), input_size_bytes_, cudaMemcpyHostToDevice, stream_),
      "cudaMemcpyAsync(H2D,input,half)");
  }

  if (!context_->enqueueV3(stream_)) {
    tools::logger()->error("[YOLO26_TRT] enqueueV3 failed");
    return {};
  }

  if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
    cuda_check(
      cudaMemcpyAsync(output_host_float_.data(), buffers_[1], output_size_bytes_, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync(D2H,output,float)");
  } else {
    cuda_check(
      cudaMemcpyAsync(output_host_half_.data(), buffers_[1], output_size_bytes_, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync(D2H,output,half)");
  }
  cuda_check(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

  if (output_dtype_ == nvinfer1::DataType::kHALF) {
    // 输出若为 FP16，统一转成 FP32 便于后续解析。
    for (size_t i = 0; i < output_host_half_.size(); ++i) {
      output_host_float_[i] = __half2float(output_host_half_[i]);
    }
  }

  const float * parse_ptr = output_host_float_.data();
  std::vector<float> transposed_output;
  if (output_channel_first_) {
    // 若导出是 [stride, rows]，转置成统一的 [rows, stride] 解析格式。
    transposed_output.resize(static_cast<size_t>(output_rows_) * output_stride_);
    for (int r = 0; r < output_rows_; ++r) {
      for (int c = 0; c < output_stride_; ++c) {
        transposed_output[r * output_stride_ + c] = parse_ptr[c * output_rows_ + r];
      }
    }
    parse_ptr = transposed_output.data();
  }

  return parse(scale, pad_x, pad_y, parse_ptr, output_rows_, output_stride_, bgr_img, tmp_img_, frame_count);
}

std::list<Armor> YOLO26_TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, 0, 0, output.ptr<float>(0), output.rows, output.cols, bgr_img, bgr_img, frame_count);
}

std::list<Armor> YOLO26_TRT::parse(
  float scale, int pad_x, int pad_y, const float * output_data, int num_rows, int stride,
  const cv::Mat & bgr_img, const cv::Mat & tmp_img, int frame_count)
{
  // parse 负责把网络原始输出转成 Armor 列表：阈值筛选 + NMS + 业务规则过滤。
  const int img_width = bgr_img.cols;
  const int img_height = bgr_img.rows;
  const float inv_scale = 1.0f / scale;
  const float pad_x_f = static_cast<float>(pad_x);
  const float pad_y_f = static_cast<float>(pad_y);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  const int estimated_valid = std::max(32, num_rows / 4);
  ids.reserve(estimated_valid);
  confidences.reserve(estimated_valid);
  boxes.reserve(estimated_valid);
  armors_key_points.reserve(estimated_valid);

  int pass_conf_count = 0;
  int invalid_box_count = 0;
  float max_conf_before_threshold = 0.0f;
  int max_conf_row = -1;
  float dbg_first_row[8] = {0.0f};
  bool dbg_has_first_row = false;

  for (int r = 0; r < num_rows; ++r) {
    const float * row_ptr = output_data + r * stride;
    if (!dbg_has_first_row) {
      const int dbg_cols = std::min(8, stride);
      for (int k = 0; k < dbg_cols; ++k) {
        dbg_first_row[k] = row_ptr[k];
      }
      dbg_has_first_row = true;
    }

    float conf = 0.0f;
    int cls = 0;

    float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;
    int keypoint_start = 6;

    if (stride == 14 || stride == 18) {
      // 格式 A：直接给出 conf、class_id、bbox、4个关键点。
      conf = row_ptr[4];
      cls = static_cast<int>(row_ptr[5]);
      x1 = row_ptr[0];
      y1 = row_ptr[1];
      x2 = row_ptr[2];
      y2 = row_ptr[3];
      keypoint_start = 6;
    } else if (stride >= 5 + class_num_ + 8) {
      // 格式 B：YOLO 风格 [obj_conf + class_probs + kpts]，需要先算最终置信度。
      const float obj_conf = row_ptr[4];
      const int cls_start = 5;
      float best_prob = 0.0f;
      int best_cls = 0;
      for (int c = 0; c < class_num_; ++c) {
        float p = row_ptr[cls_start + c];
        if (p > best_prob) {
          best_prob = p;
          best_cls = c;
        }
      }
      conf = obj_conf * best_prob;
      cls = best_cls;

      x1 = row_ptr[0];
      y1 = row_ptr[1];
      x2 = row_ptr[2];
      y2 = row_ptr[3];
      keypoint_start = 5 + class_num_;
    } else {
      continue;
    }

    if (conf > max_conf_before_threshold) {
      max_conf_before_threshold = conf;
      max_conf_row = r;
    }

    if (conf < score_threshold_) {
      continue;
    }
    pass_conf_count++;

    const int box_x1 = std::clamp(static_cast<int>((x1 - pad_x_f) * inv_scale), 0, img_width);
    const int box_y1 = std::clamp(static_cast<int>((y1 - pad_y_f) * inv_scale), 0, img_height);
    const int box_x2 = std::clamp(static_cast<int>((x2 - pad_x_f) * inv_scale), 0, img_width);
    const int box_y2 = std::clamp(static_cast<int>((y2 - pad_y_f) * inv_scale), 0, img_height);
    const int width = box_x2 - box_x1;
    const int height = box_y2 - box_y1;
    if (width <= 0 || height <= 0) {
      invalid_box_count++;
      continue;
    }

    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.reserve(4);
    for (int i = 0; i < 4; ++i) {
      armor_key_points.emplace_back(
        (row_ptr[keypoint_start + i * 2] - pad_x_f) * inv_scale,
        (row_ptr[keypoint_start + i * 2 + 1] - pad_y_f) * inv_scale);
    }

    ids.emplace_back(cls);
    confidences.emplace_back(conf);
    boxes.emplace_back(box_x1, box_y1, width, height);
    armors_key_points.emplace_back(std::move(armor_key_points));
  }

  std::vector<int> indices;
  // NMS 去除高度重叠框（可配置开关）。
  if (use_nms_) {
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);
  } else {
    indices.resize(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
  }

  std::list<Armor> armors;
  for (int i : indices) {
    sort_keypoints(armors_key_points[i]);

    if (use_roi_) {
      armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset_, YOLOVersion::YOLO26);
    } else {
      armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], YOLOVersion::YOLO26);
    }
  }

  int reject_not_armor_count = 0;
  int reject_min_conf_count = 0;
  int reject_type_count = 0;

  for (auto it = armors.begin(); it != armors.end();) {
    // 业务二次过滤：非装甲板类别、低置信度、大小类型不匹配都会剔除。
    const bool label_ok = (it->name != ArmorName::not_armor);
    const bool conf_ok = (it->confidence > min_confidence_);
    const bool type_ok = check_type(*it);

    if (!label_ok || !conf_ok || !type_ok) {
      if (!label_ok) {
        reject_not_armor_count++;
      }
      if (!conf_ok) {
        reject_min_conf_count++;
      }
      if (label_ok && conf_ok && !type_ok) {
        reject_type_count++;
      }
      it = armors.erase(it);
      continue;
    }
    it->center_norm = get_center_norm(tmp_img, it->center);
    ++it;
  }

  const bool should_log = (frame_count < 20) || (frame_count % 30 == 0) || armors.empty();
  if (should_log) {
    tools::logger()->info(
      "[YOLO26_TRT][DBG] frame={} rows={} stride={} pass_conf={} invalid_box={} nms_keep={} final_keep={} reject_not_armor={} reject_min_conf={} reject_type={} max_conf={:.4f}@row{} score_th={:.3f} min_conf={:.3f}",
      frame_count, num_rows, stride, pass_conf_count, invalid_box_count,
      indices.size(), armors.size(), reject_not_armor_count, reject_min_conf_count, reject_type_count,
      max_conf_before_threshold, max_conf_row,
      score_threshold_, min_confidence_);

    if (dbg_has_first_row) {
      tools::logger()->info(
        "[YOLO26_TRT][DBG] first_row: [{:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}]",
        dbg_first_row[0], dbg_first_row[1], dbg_first_row[2], dbg_first_row[3],
        dbg_first_row[4], dbg_first_row[5], dbg_first_row[6], dbg_first_row[7]);
    }

    int dbg_idx = 0;
    for (auto armor_it = armors.begin(); armor_it != armors.end() && dbg_idx < 3; ++armor_it, ++dbg_idx) {
      tools::logger()->info(
        "[YOLO26_TRT][DBG] keep#{} class_id={} => {} {} conf={:.3f}",
        dbg_idx, armor_it->class_id,
        COLORS[armor_it->color], ARMOR_NAMES[armor_it->name], armor_it->confidence);
    }
  }

  if (debug_ && !tmp_img.empty()) {
    draw_detections(tmp_img, armors, frame_count);
  }

  return armors;
}

bool YOLO26_TRT::check_name(const Armor & armor) const
{
  return armor.name != ArmorName::not_armor && armor.confidence > min_confidence_;
}

bool YOLO26_TRT::check_type(const Armor & armor) const
{
  // 按大小装甲板约束类别，过滤明显不合理的识别结果。
  return (armor.type == ArmorType::small)
           ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
           : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
              armor.name != ArmorName::outpost);
}

cv::Point2f YOLO26_TRT::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

void YOLO26_TRT::sort_keypoints(std::vector<cv::Point2f> & keypoints) const
{
  // 关键点统一排序为：左上、右上、右下、左下，方便后续几何计算。
  if (keypoints.size() != 4) {
    return;
  }

  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
  std::sort(bottom_points.begin(), bottom_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  keypoints[0] = top_points[0];
  keypoints[1] = top_points[1];
  keypoints[2] = bottom_points[1];
  keypoints[3] = bottom_points[0];
}

void YOLO26_TRT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  // Debug 可视化：显示置信度、颜色、类别、大小。
  cv::Mat detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::rectangle(detection, roi_, cv::Scalar(0, 255, 0), 2);
  }

  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("Yolo26_TRT Detection", detection);
}

}  // namespace auto_aim
