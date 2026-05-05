#include "classifier_trt.hpp"
#include "NvInferRuntimeBase.h"
#ifdef ENABLE_TENSORRT
#include <yaml-cpp/yaml.h>
#include "tools/path.hpp"
namespace auto_aim
{
TrtClassifier::TrtClassifier(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto model = tools::resolve_path_from_config( 
    config_path, yaml["classify_model"].as<std::string>());
    // 加载 TensorRT 引擎文件
    engine_path_ = tools::resolve_path_from_config(
      config_path, yaml["classify_engine_path"].as<std::string>(""));
    
}
void TrtClassifier::classify(auto_aim::Armor & armor)
{
  // 这里实现分类逻辑，调用 trt_classify 函数
  trt_classify(armor);
}
void TrtClassifier::trt_classify(auto_aim::Armor & armor)
{
  if (armor.pattern.empty()) {
    armor.name = ArmorName::not_armor;
    return;
  }
  // 图像预处理：转换为灰度图，调整大小为32x32，归一化等
  cv::Mat gray;
  cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

  // Resize image to 32x32
  auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  auto x_scale = static_cast<double>(32) / gray.cols;
  auto y_scale = static_cast<double>(32) / gray.rows;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(gray.rows * scale);
  auto w = static_cast<int>(gray.cols * scale);

  if (h == 0 || w == 0) {
    armor.name = ArmorName::not_armor;
    return;
  }

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(gray, input(roi), {w, h});
  // Normalize the input image to [0, 1] range
  input.convertTo(input, CV_32F, 1.0 / 255.0);

  // 初始化trt和cuda
  cudaSetDevice(0);
  cudaStreamCreate(&stream_);
  context_.reset();
  engine_.reset();
  runtime_.reset();
  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (engine_path_.empty()) {
    throw std::runtime_error("TensorRT engine path is empty");
  }
  // 加载引擎 反序列化引擎文件
  std::ifstream engine_file(engine_path_, std::ios::binary);
  if (!engine_file.good()) {
    throw std::runtime_error("Failed to open engine file: " + engine_path_);
  }
  engine_file.seekg(0, std::ios::end);
  size_t size = engine_file.tellg();
  engine_file.seekg(0, std::ios::beg);
  std::vector<char> engine_data(size);
  engine_file.read(engine_data.data(), size);
  engine_file.close();
  engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), size));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize TensorRT engine");
  }
  // 创建执行上下文
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create TensorRT execution context");
  }
  // 分配输入输出缓冲区
  cudaMalloc(&buffers_[0], 32 * 32 * sizeof(float)); // 输入
  cudaMalloc(&buffers_[1], 9 * sizeof(float)); // 输出
  // 将预处理后的图像数据复制到输入缓冲区
  cudaMemcpyAsync(buffers_[0], input.data, 32 * 32 * sizeof(float), cudaMemcpyHostToDevice, stream_);
  // 设置输入输出tensor地址
  context_->setTensorAddress(engine_->getIOTensorName(0), buffers_[0]);
  context_->setTensorAddress(engine_->getIOTensorName(1), buffers_[1]);
  // 执行推理
  context_->enqueueV3(stream_ );
  // 从输出缓冲区复制结果回主机
  float output[9];
  cudaMemcpyAsync(output, buffers_[1], 9 * sizeof(float), cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);
  // softmax
  float max = *std::max_element(output, output + 9);
  float sum = 0.0f;
  for (int i = 0; i < 9; ++i) {
    output[i] = exp(output[i] - max);
    sum += output[i];
  }
  if (sum > 0.0f) {
    for (int i = 0; i < 9; ++i) {
      output[i] /= sum;
    }
  }
  // 找到最大confidence
  double confidence;
  cv::Point label_point;
  cv::Mat outputs(1, 9, CV_32F, output);
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
  int label_id = label_point.x;

  armor.confidence = confidence;
  armor.name = static_cast<ArmorName>(label_id);

  cudaFree(buffers_[0]);
  cudaFree(buffers_[1]);
  cudaStreamDestroy(stream_);
  
}
#endif  // ENABLE_TENSORRT
}