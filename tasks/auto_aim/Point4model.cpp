#include "Point4model.hpp"

#ifdef ENABLE_TENSORRT

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/path.hpp"

namespace auto_aim
{
namespace
{
auto runtime_deleter_point4 = [](nvinfer1::IRuntime * p) { if (p) delete p; };
auto engine_deleter_point4 = [](nvinfer1::ICudaEngine * p) { if (p) delete p; };
auto context_deleter_point4 = [](nvinfer1::IExecutionContext * p) { if (p) delete p; };
}  // namespace

Point4ModelTRT::Point4ModelTRT(const std::string & config_path, bool debug)
: debug_(debug),
  runtime_(nullptr, runtime_deleter_point4),
  engine_(nullptr, engine_deleter_point4),
  context_(nullptr, context_deleter_point4),
  detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  engine_path_ = tools::resolve_path_from_config(
    config_path, yaml["point4_engine_path"].as<std::string>(""));
  onnx_path_ = tools::resolve_path_from_config(
    config_path, yaml["point4_onnx_path"].as<std::string>(""));
  device_ = yaml["device"].as<std::string>("GPU");
  score_threshold_ = yaml["point4_score_threshold"].as<float>(0.65f);
  nms_threshold_ = yaml["point4_nms_threshold"].as<float>(0.45f);
  min_confidence_ = yaml["min_confidence"].as<double>(score_threshold_);
  keep_ratio_ = yaml["point4_keep_ratio"].as<bool>(true);
  filter_enemy_color_ = yaml["point4_filter_enemy_color"].as<bool>(true);
  swap_color_id_ = yaml["point4_swap_color_id"].as<bool>(false);
  use_traditional_ = yaml["use_traditional"].as<bool>(false);

  const auto enemy_color = yaml["enemy_color"].as<std::string>("");
  if (enemy_color == "blue") {
    detect_color_ = 0;
  } else if (enemy_color == "red") {
    detect_color_ = 1;
  }

  int x = yaml["roi"]["x"].as<int>(0);
  int y = yaml["roi"]["y"].as<int>(0);
  int width = yaml["roi"]["width"].as<int>(-1);
  int height = yaml["roi"]["height"].as<int>(-1);
  use_roi_ = yaml["use_roi"].as<bool>(false);
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  cudaSetDevice(0);
  cudaStreamCreate(&stream_);

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("[Point4ModelTRT] Failed to create TensorRT runtime");
  }

  const bool engine_exists = !engine_path_.empty() && std::filesystem::exists(engine_path_);
  const bool onnx_exists = !onnx_path_.empty() && std::filesystem::exists(onnx_path_);
  if (engine_exists) {
    load_engine(engine_path_);
  } else if (onnx_exists) {
    build_engine_from_onnx(onnx_path_);
    if (!engine_path_.empty()) {
      serialize_engine(engine_path_);
    }
  } else {
    throw std::runtime_error(
      "[Point4ModelTRT] No valid model found. Set point4_engine_path or point4_onnx_path.");
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("[Point4ModelTRT] Failed to create execution context");
  }

  const char * input_name = engine_->getIOTensorName(0);
  const char * output_name = engine_->getIOTensorName(1);
  auto input_dims = engine_->getTensorShape(input_name);
  input_dtype_ = engine_->getTensorDataType(input_name);
  output_dtype_ = engine_->getTensorDataType(output_name);

  if (input_dims.nbDims != 4) {
    throw std::runtime_error("[Point4ModelTRT] Expected 4D input tensor");
  }

  if (input_dims.d[1] == 3 || input_dims.d[1] == 1 || input_dims.d[1] <= 0) {
    input_nhwc_ = false;
    if (input_dims.d[2] > 0) input_h_ = input_dims.d[2];
    if (input_dims.d[3] > 0) input_w_ = input_dims.d[3];
  } else if (input_dims.d[3] == 3 || input_dims.d[3] == 1 || input_dims.d[3] <= 0) {
    input_nhwc_ = true;
    if (input_dims.d[1] > 0) input_h_ = input_dims.d[1];
    if (input_dims.d[2] > 0) input_w_ = input_dims.d[2];
  } else {
    throw std::runtime_error("[Point4ModelTRT] Unsupported input tensor layout");
  }

  bool dynamic_input = false;
  for (int i = 0; i < input_dims.nbDims; ++i) {
    if (input_dims.d[i] <= 0) dynamic_input = true;
  }
  if (dynamic_input) {
    nvinfer1::Dims4 fixed_input_shape = input_nhwc_
      ? nvinfer1::Dims4{1, input_h_, input_w_, 3}
      : nvinfer1::Dims4{1, 3, input_h_, input_w_};
    if (!context_->setInputShape(input_name, fixed_input_shape)) {
      throw std::runtime_error("[Point4ModelTRT] Failed to set dynamic input shape");
    }
  }

  prepare_tensor_layouts(input_name, output_name);

  const size_t input_elements = input_nhwc_
    ? static_cast<size_t>(1) * input_h_ * input_w_ * 3
    : static_cast<size_t>(1) * 3 * input_h_ * input_w_;
  input_size_bytes_ = input_elements * dtype_size(input_dtype_);

  size_t output_elements = static_cast<size_t>(output_rows_) * output_stride_;
  output_size_bytes_ = output_elements * dtype_size(output_dtype_);

  if (input_dtype_ == nvinfer1::DataType::kFLOAT) {
    input_host_float_.resize(input_elements);
  } else if (input_dtype_ == nvinfer1::DataType::kHALF) {
    input_host_half_.resize(input_elements);
  } else {
    throw std::runtime_error("[Point4ModelTRT] Only FP32/FP16 input tensors are supported");
  }

  if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
    output_host_float_.resize(output_elements);
  } else if (output_dtype_ == nvinfer1::DataType::kHALF) {
    output_host_half_.resize(output_elements);
    output_host_float_.resize(output_elements);
  } else {
    throw std::runtime_error("[Point4ModelTRT] Only FP32/FP16 output tensors are supported");
  }

  cudaMalloc(reinterpret_cast<void **>(&buffers_[0]), input_size_bytes_);
  cudaMalloc(reinterpret_cast<void **>(&buffers_[1]), output_size_bytes_);
  context_->setTensorAddress(input_name, buffers_[0]);
  context_->setTensorAddress(output_name, buffers_[1]);

  tools::logger()->info(
    "[Point4ModelTRT] initialized input={}x{} layout={} output rows={} stride={} score_th={:.2f}",
    input_w_, input_h_, input_nhwc_ ? "NHWC" : "NCHW", output_rows_, output_stride_,
    score_threshold_);
}

Point4ModelTRT::~Point4ModelTRT()
{
  if (buffers_[0]) cudaFree(buffers_[0]);
  if (buffers_[1]) cudaFree(buffers_[1]);
  if (stream_) cudaStreamDestroy(stream_);
}

void Point4ModelTRT::load_engine(const std::string & engine_file)
{
  std::ifstream file(engine_file, std::ios::binary);
  if (!file.good()) {
    throw std::runtime_error("[Point4ModelTRT] Failed to open engine: " + engine_file);
  }
  file.seekg(0, std::ios::end);
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  file.read(data.data(), size);
  engine_.reset(runtime_->deserializeCudaEngine(data.data(), size));
  if (!engine_) {
    throw std::runtime_error("[Point4ModelTRT] Failed to deserialize engine");
  }
}

void Point4ModelTRT::build_engine_from_onnx(const std::string & onnx_file)
{
  auto builder = std::unique_ptr<nvinfer1::IBuilder, void (*)(nvinfer1::IBuilder *)>(
    nvinfer1::createInferBuilder(logger_), [](nvinfer1::IBuilder * p) { if (p) delete p; });
  const auto flags =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition, void (*)(nvinfer1::INetworkDefinition *)>(
    builder->createNetworkV2(flags), [](nvinfer1::INetworkDefinition * p) { if (p) delete p; });
  auto parser = std::unique_ptr<nvonnxparser::IParser, void (*)(nvonnxparser::IParser *)>(
    nvonnxparser::createParser(*network, logger_), [](nvonnxparser::IParser * p) { if (p) delete p; });

  if (!parser->parseFromFile(onnx_file.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
    throw std::runtime_error("[Point4ModelTRT] Failed to parse ONNX: " + onnx_file);
  }

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig, void (*)(nvinfer1::IBuilderConfig *)>(
    builder->createBuilderConfig(), [](nvinfer1::IBuilderConfig * p) { if (p) delete p; });
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);
  if (builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  auto serialized = std::unique_ptr<nvinfer1::IHostMemory, void (*)(nvinfer1::IHostMemory *)>(
    builder->buildSerializedNetwork(*network, *config), [](nvinfer1::IHostMemory * p) { if (p) delete p; });
  if (!serialized) {
    throw std::runtime_error("[Point4ModelTRT] Failed to build TensorRT engine");
  }
  engine_.reset(runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));
  if (!engine_) {
    throw std::runtime_error("[Point4ModelTRT] Failed to deserialize built engine");
  }
}

void Point4ModelTRT::serialize_engine(const std::string & engine_file)
{
  auto serialized = std::unique_ptr<nvinfer1::IHostMemory, void (*)(nvinfer1::IHostMemory *)>(
    engine_->serialize(), [](nvinfer1::IHostMemory * p) { if (p) delete p; });
  if (!serialized) return;

  const auto parent = std::filesystem::path(engine_file).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
  std::ofstream file(engine_file, std::ios::binary);
  if (!file.good()) return;
  file.write(reinterpret_cast<const char *>(serialized->data()), serialized->size());
}

void Point4ModelTRT::prepare_tensor_layouts(const char * input_name, const char * output_name)
{
  auto runtime_input_dims = context_->getTensorShape(input_name);
  auto runtime_output_dims = context_->getTensorShape(output_name);

  if (input_nhwc_) {
    if (runtime_input_dims.d[1] > 0) input_h_ = runtime_input_dims.d[1];
    if (runtime_input_dims.d[2] > 0) input_w_ = runtime_input_dims.d[2];
  } else {
    if (runtime_input_dims.d[2] > 0) input_h_ = runtime_input_dims.d[2];
    if (runtime_input_dims.d[3] > 0) input_w_ = runtime_input_dims.d[3];
  }

  if (runtime_output_dims.nbDims == 3) {
    if (runtime_output_dims.d[2] == 22) {
      output_rows_ = runtime_output_dims.d[1];
      output_stride_ = runtime_output_dims.d[2];
      output_channel_first_ = false;
      return;
    }
    if (runtime_output_dims.d[1] == 22) {
      output_rows_ = runtime_output_dims.d[2];
      output_stride_ = runtime_output_dims.d[1];
      output_channel_first_ = true;
      return;
    }
  }

  if (runtime_output_dims.nbDims == 2) {
    if (runtime_output_dims.d[1] == 22) {
      output_rows_ = runtime_output_dims.d[0];
      output_stride_ = runtime_output_dims.d[1];
      output_channel_first_ = false;
      return;
    }
    if (runtime_output_dims.d[0] == 22) {
      output_rows_ = runtime_output_dims.d[1];
      output_stride_ = runtime_output_dims.d[0];
      output_channel_first_ = true;
      return;
    }
  }

  throw std::runtime_error("[Point4ModelTRT] Unsupported output shape, expected stride 22");
}

void Point4ModelTRT::preprocess(
  const cv::Mat & bgr_img, float & x_scale, float & y_scale, int & resized_w, int & resized_h)
{
  cv::Mat input(input_h_, input_w_, CV_8UC3, cv::Scalar(0, 0, 0));
  if (keep_ratio_) {
    const float scale =
      std::min(static_cast<float>(input_h_) / bgr_img.rows, static_cast<float>(input_w_) / bgr_img.cols);
    resized_w = static_cast<int>(bgr_img.cols * scale);
    resized_h = static_cast<int>(bgr_img.rows * scale);
    x_scale = scale;
    y_scale = scale;
    cv::resize(bgr_img, input(cv::Rect(0, 0, resized_w, resized_h)), {resized_w, resized_h});
  } else {
    resized_w = input_w_;
    resized_h = input_h_;
    x_scale = static_cast<float>(input_w_) / bgr_img.cols;
    y_scale = static_cast<float>(input_h_) / bgr_img.rows;
    cv::resize(bgr_img, input, {input_w_, input_h_});
  }

  const int channel_map[3] = {2, 1, 0};  // BGR -> RGB, then /255, matching OpenVINO preprocess.
  if (input_dtype_ == nvinfer1::DataType::kFLOAT) {
    if (input_nhwc_) {
      for (int h = 0; h < input_h_; ++h) {
        for (int w = 0; w < input_w_; ++w) {
          const auto pixel = input.at<cv::Vec3b>(h, w);
          const size_t base = (static_cast<size_t>(h) * input_w_ + w) * 3;
          for (int c = 0; c < 3; ++c) input_host_float_[base + c] = pixel[channel_map[c]] / 255.0f;
        }
      }
    } else {
      const size_t plane = static_cast<size_t>(input_h_) * input_w_;
      for (int c = 0; c < 3; ++c) {
        for (int h = 0; h < input_h_; ++h) {
          for (int w = 0; w < input_w_; ++w) {
            input_host_float_[c * plane + h * input_w_ + w] =
              input.at<cv::Vec3b>(h, w)[channel_map[c]] / 255.0f;
          }
        }
      }
    }
  } else {
    if (input_nhwc_) {
      for (int h = 0; h < input_h_; ++h) {
        for (int w = 0; w < input_w_; ++w) {
          const auto pixel = input.at<cv::Vec3b>(h, w);
          const size_t base = (static_cast<size_t>(h) * input_w_ + w) * 3;
          for (int c = 0; c < 3; ++c) {
            input_host_half_[base + c] = __float2half(pixel[channel_map[c]] / 255.0f);
          }
        }
      }
    } else {
      const size_t plane = static_cast<size_t>(input_h_) * input_w_;
      for (int c = 0; c < 3; ++c) {
        for (int h = 0; h < input_h_; ++h) {
          for (int w = 0; w < input_w_; ++w) {
            input_host_half_[c * plane + h * input_w_ + w] =
              __float2half(input.at<cv::Vec3b>(h, w)[channel_map[c]] / 255.0f);
          }
        }
      }
    }
  }
}

std::list<Armor> Point4ModelTRT::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("[Point4ModelTRT] Empty image");
    return {};
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols - roi_.x;
    if (roi_.height == -1) roi_.height = raw_img.rows - roi_.y;
    roi_ &= cv::Rect(0, 0, raw_img.cols, raw_img.rows);
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }
  tmp_img_ = raw_img;

  float x_scale = 1.0f;
  float y_scale = 1.0f;
  int resized_w = input_w_;
  int resized_h = input_h_;
  preprocess(bgr_img, x_scale, y_scale, resized_w, resized_h);

  if (input_dtype_ == nvinfer1::DataType::kFLOAT) {
    cudaMemcpyAsync(buffers_[0], input_host_float_.data(), input_size_bytes_, cudaMemcpyHostToDevice, stream_);
  } else {
    cudaMemcpyAsync(buffers_[0], input_host_half_.data(), input_size_bytes_, cudaMemcpyHostToDevice, stream_);
  }

  if (!context_->enqueueV3(stream_)) {
    tools::logger()->error("[Point4ModelTRT] enqueueV3 failed");
    return {};
  }

  if (output_dtype_ == nvinfer1::DataType::kFLOAT) {
    cudaMemcpyAsync(output_host_float_.data(), buffers_[1], output_size_bytes_, cudaMemcpyDeviceToHost, stream_);
  } else {
    cudaMemcpyAsync(output_host_half_.data(), buffers_[1], output_size_bytes_, cudaMemcpyDeviceToHost, stream_);
  }
  cudaStreamSynchronize(stream_);

  if (output_dtype_ == nvinfer1::DataType::kHALF) {
    for (size_t i = 0; i < output_host_half_.size(); ++i) {
      output_host_float_[i] = __half2float(output_host_half_[i]);
    }
  }

  const float * parse_data = output_host_float_.data();
  std::vector<float> transposed;
  if (output_channel_first_) {
    transposed.resize(static_cast<size_t>(output_rows_) * output_stride_);
    for (int r = 0; r < output_rows_; ++r) {
      for (int c = 0; c < output_stride_; ++c) {
        transposed[r * output_stride_ + c] = parse_data[c * output_rows_ + r];
      }
    }
    parse_data = transposed.data();
  }

  return parse(parse_data, output_rows_, output_stride_, x_scale, y_scale, raw_img, frame_count);
}

std::list<Armor> Point4ModelTRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(
    output.ptr<float>(0), output.rows, output.cols, static_cast<float>(scale),
    static_cast<float>(scale), bgr_img, frame_count);
}

std::list<Armor> Point4ModelTRT::parse(
  const float * output_data, int rows, int stride, float x_scale, float y_scale,
  const cv::Mat & raw_img, int frame_count)
{
  std::vector<Detection> detections;
  detections.reserve(std::max(32, rows / 8));
  std::vector<cv::Rect> boxes;
  std::vector<float> nms_scores;

  for (int r = 0; r < rows; ++r) {
    const float * row = output_data + r * stride;
    const float object_conf = static_cast<float>(sigmoid(row[8]));
    if (object_conf < score_threshold_) continue;

    int color_id = 0;
    float color_score = row[9];
    for (int i = 1; i < 4; ++i) {
      if (row[9 + i] > color_score) {
        color_score = row[9 + i];
        color_id = i;
      }
    }
    if (color_id == 2 || color_id == 3) continue;
    if (swap_color_id_) color_id = 1 - color_id;
    if (filter_enemy_color_ && detect_color_ >= 0 && color_id != detect_color_) continue;

    int num_id = 0;
    float num_score = row[13];
    for (int i = 1; i < 9; ++i) {
      if (row[13 + i] > num_score) {
        num_score = row[13 + i];
        num_id = i;
      }
    }

    std::vector<cv::Point2f> points;
    points.reserve(4);
    points.emplace_back(row[0] / x_scale, row[1] / y_scale);
    points.emplace_back(row[6] / x_scale, row[7] / y_scale);
    points.emplace_back(row[4] / x_scale, row[5] / y_scale);
    points.emplace_back(row[2] / x_scale, row[3] / y_scale);

    float min_x = points[0].x;
    float max_x = points[0].x;
    float min_y = points[0].y;
    float max_y = points[0].y;
    for (size_t i = 1; i < points.size(); ++i) {
      min_x = std::min(min_x, points[i].x);
      max_x = std::max(max_x, points[i].x);
      min_y = std::min(min_y, points[i].y);
      max_y = std::max(max_y, points[i].y);
    }

    cv::Rect box(
      std::clamp(static_cast<int>(min_x), 0, raw_img.cols),
      std::clamp(static_cast<int>(min_y), 0, raw_img.rows),
      std::clamp(static_cast<int>(max_x - min_x), 0, raw_img.cols),
      std::clamp(static_cast<int>(max_y - min_y), 0, raw_img.rows));
    if (box.width <= 0 || box.height <= 0) continue;

    detections.push_back({color_id, num_id, object_conf, num_score, box, points});
    boxes.push_back(box);
    nms_scores.push_back(num_score);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, nms_scores, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (int i : indices) {
    const auto & det = detections[i];
    if (use_roi_) {
      armors.emplace_back(
        det.color_id, det.num_id, det.object_conf, det.box, det.points, offset_);
    } else {
      armors.emplace_back(det.color_id, det.num_id, det.object_conf, det.box, det.points);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it) || !check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    if (use_traditional_) detector_.detect(*it, raw_img);
    it->center_norm = get_center_norm(raw_img, it->center);
    ++it;
  }

  if (debug_) {
    draw_detections(raw_img, armors, frame_count);
  }

  return armors;
}

bool Point4ModelTRT::check_name(const Armor & armor) const
{
  return armor.name != ArmorName::not_armor && armor.confidence > min_confidence_;
}

bool Point4ModelTRT::check_type(const Armor & armor) const
{
  return (armor.type == ArmorType::small)
           ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
           : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
              armor.name != ArmorName::outpost);
}

cv::Point2f Point4ModelTRT::get_center_norm(
  const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  return {center.x / bgr_img.cols, center.y / bgr_img.rows};
}

void Point4ModelTRT::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
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
  // cv::imshow("Point4ModelTRT Detection", detection);
}

double Point4ModelTRT::sigmoid(double x)
{
  if (x > 0) return 1.0 / (1.0 + std::exp(-x));
  const double exp_x = std::exp(x);
  return exp_x / (1.0 + exp_x);
}

size_t Point4ModelTRT::dtype_size(nvinfer1::DataType dtype)
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

}  // namespace auto_aim

#endif  // ENABLE_TENSORRT
