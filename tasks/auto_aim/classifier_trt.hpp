#ifndef AUTO_AIM__CLASSIFIER_TRT_HPP
#define AUTO_AIM__CLASSIFIER_TRT_HPP

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <fstream>
#include <vector>
#include <memory>

#include "armor.hpp"
#include "tasks/auto_aim/yolos/tensorrt_logger.hpp"

#include <opencv2/opencv.hpp>

#ifdef ENABLE_TENSORRT  // 仅在启用 TensorRT 时条件编译
namespace auto_aim {

class TrtClassifier
{
public:
  explicit TrtClassifier(const std::string & config_path);
  void classify(auto_aim::Armor & armor); // 提供给 Detector 使用的分类接口
  void trt_classify(auto_aim::Armor & armor); // 直接使用 TensorRT 进行分类的内部函数
  
private:
  // TensorRT 相关成员变量
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  // 输入输出缓冲区指针
  void *buffers_[2]; // 输入和输出的 GPU 内存指针
  int input_index_;
  int output_index_;
  cv::Mat input_image_; // 用于存储预处理后的输入图像
  cudaStream_t stream_; // CUDA 流，用于异步执行
  // 文件路径
  std::string engine_path_;
  Logger logger_; // TensorRT 专用 Logger，内部调用 tools::logger()
};
#endif  // ENABLE_TENSORRT
#endif  // AUTO_AIM__CLASSIFIER_TRT_HPP 
}