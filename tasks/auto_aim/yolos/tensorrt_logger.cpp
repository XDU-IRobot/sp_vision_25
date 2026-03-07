#include "tensorrt_logger.hpp"

#include "tools/logger.hpp"

namespace auto_aim
{

void Logger::log(Severity severity, const char * msg) noexcept
{
  if (severity <= Severity::kWARNING) {
    tools::logger()->warn("[TensorRT] {}", msg);
  }
}

}  // namespace auto_aim
