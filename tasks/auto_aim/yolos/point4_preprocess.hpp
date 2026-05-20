#ifndef AUTO_AIM__POINT4_PREPROCESS_HPP
#define AUTO_AIM__POINT4_PREPROCESS_HPP

#include <cuda_runtime.h>

namespace auto_aim
{

void point4_preprocess_cuda(
  const unsigned char * src_device, int src_width, int src_height, void * dst_device,
  int dst_width, int dst_height, bool keep_ratio, bool input_nhwc, bool output_fp16,
  cudaStream_t stream);

}  // namespace auto_aim

#endif  // AUTO_AIM__POINT4_PREPROCESS_HPP
