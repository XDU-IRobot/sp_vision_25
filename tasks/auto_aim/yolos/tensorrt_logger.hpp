#ifndef AUTO_AIM__TENSORRT_LOGGER_HPP
#define AUTO_AIM__TENSORRT_LOGGER_HPP

#include <NvInfer.h>

namespace auto_aim
{

// TensorRT Logger实现
// 继承自nvinfer1::ILogger，内部调用tools::logger()进行实际日志输出
class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TENSORRT_LOGGER_HPP
