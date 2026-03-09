#include "yolo11_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <vector>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/path.hpp"

namespace auto_aim
{

// 自定义删除器
auto runtime_deleter = [](nvinfer1::IRuntime* p) { if(p) delete p; };
auto engine_deleter = [](nvinfer1::ICudaEngine* p) { if(p) delete p; };
auto context_deleter = [](nvinfer1::IExecutionContext* p) { if(p) delete p; };

YOLO11_TRT::YOLO11_TRT(const std::string & config_path, bool debug)
: debug_(debug),
  detector_(config_path, false),
  runtime_(nullptr, runtime_deleter),
  engine_(nullptr, engine_deleter),
  context_(nullptr, context_deleter)
{
  auto yaml = YAML::LoadFile(config_path);

  // 读取配置
  engine_path_ = tools::resolve_path_from_config(
    config_path, yaml["yolo11_engine_path"].as<std::string>(""));
  onnx_path_ = tools::resolve_path_from_config(
    config_path, yaml["yolo11_onnx_path"].as<std::string>(""));
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
    tools::logger()->info("[YOLO11_TRT] Loading engine from: {}", engine_path_);
    loadEngine(engine_path_);
  } else if (!onnx_path_.empty() && std::filesystem::exists(onnx_path_)) {
    tools::logger()->info("[YOLO11_TRT] Building engine from ONNX: {}", onnx_path_);
    buildEngineFromONNX(onnx_path_);
    if (!engine_path_.empty()) {
      serializeEngine(engine_path_);
    }
  } else {
    throw std::runtime_error(
      "No valid model found! Please provide either:\n"
      "  - yolo11_engine_path: path to .engine file\n"
      "  - yolo11_onnx_path: path to .onnx file");
  }

  // 创建执行上下文
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create execution context");
  }

  // 获取输入输出维度（TensorRT 10.x使用新API）
  auto input_dims = engine_->getTensorShape(engine_->getIOTensorName(0));
  auto output_dims = engine_->getTensorShape(engine_->getIOTensorName(1));

  input_w_ = input_dims.d[3];
  input_h_ = input_dims.d[2];
  output_num_detections_ = output_dims.d[1];
  output_data_size_ = output_dims.d[2];

  // 计算缓冲区大小
  input_size_ = 1 * 3 * input_h_ * input_w_ * sizeof(float);
  output_size_ = 1 * output_num_detections_ * output_data_size_ * sizeof(float);

  // 分配GPU内存
  cudaMalloc(&buffers_[0], input_size_);
  cudaMalloc(&buffers_[1], output_size_);

  // 分配CPU内存
  output_host_ = new float[output_num_detections_ * output_data_size_];

  // 预分配输入图像
  input_image_ = cv::Mat(input_h_, input_w_, CV_8UC3, cv::Scalar(0, 0, 0));

  // 设置输入输出tensor地址（TensorRT 10.x）
  context_->setTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
  context_->setTensorAddress(engine_->getIOTensorName(1), buffers_[1]);

  tools::logger()->info(
    "[YOLO11_TRT] Initialized: {}x{} input, {} max detections",
    input_w_, input_h_, output_num_detections_);
}

YOLO11_TRT::~YOLO11_TRT()
{
  delete[] output_host_;
  cudaFree(buffers_[0]);
  cudaFree(buffers_[1]);
  cudaStreamDestroy(stream_);
}

void YOLO11_TRT::loadEngine(const std::string & engine_file)
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

  tools::logger()->info("[YOLO11_TRT] Engine loaded successfully");
}

void YOLO11_TRT::buildEngineFromONNX(const std::string & onnx_file)
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

  // TensorRT 10.x使用新API设置内存
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

  tools::logger()->info("[YOLO11_TRT] Engine built successfully");
}

void YOLO11_TRT::serializeEngine(const std::string & engine_file)
{
  auto serialized = std::unique_ptr<nvinfer1::IHostMemory, void(*)(nvinfer1::IHostMemory*)>(
    engine_->serialize(), [](nvinfer1::IHostMemory* p) { if(p) delete p; });

  if (!serialized) {
    tools::logger()->error("[YOLO11_TRT] Failed to serialize engine");
    return;
  }

  std::ofstream file(engine_file, std::ios::binary);
  if (!file.good()) {
    tools::logger()->error("[YOLO11_TRT] Failed to open file for writing: {}", engine_file);
    return;
  }

  file.write(reinterpret_cast<const char *>(serialized->data()), serialized->size());
  file.close();

  tools::logger()->info("[YOLO11_TRT] Engine serialized to: {}", engine_file);
}

std::list<Armor> YOLO11_TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  cv::Mat bgr_img;
  if (use_roi_) {
    bgr_img = raw_img(roi_).clone();
  } else {
    bgr_img = raw_img;
  }

  // Letterbox resize
  float scale = std::min(
    static_cast<float>(input_w_) / bgr_img.cols, static_cast<float>(input_h_) / bgr_img.rows);
  int new_w = static_cast<int>(bgr_img.cols * scale);
  int new_h = static_cast<int>(bgr_img.rows * scale);
  int pad_x = (input_w_ - new_w) / 2;
  int pad_y = (input_h_ - new_h) / 2;

  cv::Mat resized;
  cv::resize(bgr_img, resized, cv::Size(new_w, new_h));

  input_image_ = cv::Scalar(0, 0, 0);
  resized.copyTo(input_image_(cv::Rect(pad_x, pad_y, new_w, new_h)));
  cv::cvtColor(input_image_, input_image_, cv::COLOR_BGR2RGB);

  // 转换为NCHW格式并归一化
  std::vector<float> input_data(3 * input_h_ * input_w_);
  for (int c = 0; c < 3; ++c) {
    for (int h = 0; h < input_h_; ++h) {
      for (int w = 0; w < input_w_; ++w) {
        input_data[c * input_h_ * input_w_ + h * input_w_ + w] =
          input_image_.at<cv::Vec3b>(h, w)[c] / 255.0f;
      }
    }
  }

  // 异步推理
  if (use_async_inference_ && !first_frame_) {
    cudaStreamSynchronize(stream_);
    cudaMemcpyAsync(output_host_, buffers_[1], output_size_, cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    cudaMemcpyAsync(buffers_[0], input_data.data(), input_size_, cudaMemcpyHostToDevice, stream_);
    context_->enqueueV3(stream_);

    cv::Mat output(output_num_detections_, output_data_size_, CV_32F, output_host_);
    auto result = parse(prev_scale_, prev_pad_x_, prev_pad_y_, output, prev_raw_img_, prev_frame_count_);

    prev_raw_img_ = bgr_img.clone();
    prev_scale_ = scale;
    prev_pad_x_ = pad_x;
    prev_pad_y_ = pad_y;
    prev_frame_count_ = frame_count;

    return result;
  } else {
    // 同步推理
    cudaMemcpyAsync(buffers_[0], input_data.data(), input_size_, cudaMemcpyHostToDevice, stream_);
    context_->enqueueV3(stream_);
    cudaMemcpyAsync(output_host_, buffers_[1], output_size_, cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    cv::Mat output(output_num_detections_, output_data_size_, CV_32F, output_host_);
    auto result = parse(scale, pad_x, pad_y, output, bgr_img, frame_count);

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

std::list<Armor> YOLO11_TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, 0, 0, output, bgr_img, frame_count);
}

std::list<Armor> YOLO11_TRT::parse(
  float scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 解析YOLO输出：[num_detections, 18]
  std::vector<cv::Rect> boxes;  // 注意：使用Rect而不是Rect2f
  std::vector<float> confidences;
  std::vector<int> class_ids;
  std::vector<std::vector<cv::Point2f>> keypoints_list;

  const float inv_scale = 1.0f / scale;

  for (int i = 0; i < output.rows; ++i) {
    float conf = output.at<float>(i, 4);
    if (conf < score_threshold_) continue;

    int class_id = static_cast<int>(output.at<float>(i, 5));
    float x1 = (output.at<float>(i, 0) - pad_x) * inv_scale;
    float y1 = (output.at<float>(i, 1) - pad_y) * inv_scale;
    float x2 = (output.at<float>(i, 2) - pad_x) * inv_scale;
    float y2 = (output.at<float>(i, 3) - pad_y) * inv_scale;

    cv::Rect box(
      static_cast<int>(x1),
      static_cast<int>(y1),
      static_cast<int>(x2 - x1),
      static_cast<int>(y2 - y1)
    );

    // 提取4个关键点
    std::vector<cv::Point2f> kpts;
    for (int k = 0; k < 4; ++k) {
      float kx = (output.at<float>(i, 6 + k * 3) - pad_x) * inv_scale;
      float ky = (output.at<float>(i, 6 + k * 3 + 1) - pad_y) * inv_scale;
      kpts.emplace_back(kx, ky);
    }

    boxes.push_back(box);
    confidences.push_back(conf);
    class_ids.push_back(class_id);
    keypoints_list.push_back(kpts);
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  // 构造Armor列表
  std::list<Armor> armors;
  for (int idx : indices) {
    auto & kpts = keypoints_list[idx];

    // 如果使用ROI，调整关键点坐标
    if (use_roi_) {
      for (auto & pt : kpts) {
        pt += offset_;
      }
      boxes[idx].x += static_cast<int>(offset_.x);
      boxes[idx].y += static_cast<int>(offset_.y);
    }

    // 使用正确的Armor构造函数（YOLO11版本）
    try {
      Armor armor(
        class_ids[idx],
        confidences[idx],
        boxes[idx],
        kpts,
        YOLOVersion::YOLO11
      );

      if (!check_name(armor) || !check_type(armor)) {
        continue;
      }

      armors.push_back(armor);
    } catch (const std::exception& e) {
      tools::logger()->warn("[YOLO11_TRT] Failed to create armor: {}", e.what());
      continue;
    }
  }

  if (debug_) {
    draw_detections(bgr_img, armors, frame_count);
  }

  return armors;
}

bool YOLO11_TRT::check_name(const Armor & armor) const
{
  return armor.confidence >= min_confidence_;
}

bool YOLO11_TRT::check_type(const Armor & armor) const
{
  if (armor.points.size() != 4) return false;

  float width = cv::norm(armor.points[1] - armor.points[0]);
  float height = cv::norm(armor.points[2] - armor.points[1]);
  float ratio = width / std::max(height, 1.0f);

  return ratio > 0.5f && ratio < 5.0f;
}

void YOLO11_TRT::draw_detections(
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
