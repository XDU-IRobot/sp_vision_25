#include "point4_preprocess.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace auto_aim
{
namespace
{

__device__ __forceinline__ int clamp_int_point4(int v, int lo, int hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

__device__ __forceinline__ float lerp_point4(float a, float b, float t)
{
  return a + (b - a) * t;
}

__device__ __forceinline__ float read_bilinear_channel_point4(
  const unsigned char * src, int src_width, int src_height, float src_x, float src_y, int channel)
{
  src_x = fminf(fmaxf(src_x, 0.0f), static_cast<float>(src_width - 1));
  src_y = fminf(fmaxf(src_y, 0.0f), static_cast<float>(src_height - 1));

  const int x0 = clamp_int_point4(static_cast<int>(floorf(src_x)), 0, src_width - 1);
  const int y0 = clamp_int_point4(static_cast<int>(floorf(src_y)), 0, src_height - 1);
  const int x1 = clamp_int_point4(x0 + 1, 0, src_width - 1);
  const int y1 = clamp_int_point4(y0 + 1, 0, src_height - 1);
  const float tx = src_x - x0;
  const float ty = src_y - y0;

  const float v00 = static_cast<float>(src[(y0 * src_width + x0) * 3 + channel]);
  const float v01 = static_cast<float>(src[(y0 * src_width + x1) * 3 + channel]);
  const float v10 = static_cast<float>(src[(y1 * src_width + x0) * 3 + channel]);
  const float v11 = static_cast<float>(src[(y1 * src_width + x1) * 3 + channel]);

  const float top = lerp_point4(v00, v01, tx);
  const float bottom = lerp_point4(v10, v11, tx);
  return lerp_point4(top, bottom, ty);
}

__device__ __forceinline__ void write_point4_value(
  void * dst, int dst_idx, float value, bool output_fp16)
{
  if (output_fp16) {
    reinterpret_cast<__half *>(dst)[dst_idx] = __float2half(value);
  } else {
    reinterpret_cast<float *>(dst)[dst_idx] = value;
  }
}

__global__ void point4_preprocess_kernel(
  const unsigned char * src, int src_width, int src_height, void * dst, int dst_width,
  int dst_height, bool keep_ratio, bool input_nhwc, bool output_fp16)
{
  const int dst_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int dst_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (dst_x >= dst_width || dst_y >= dst_height) return;

  float src_x = 0.0f;
  float src_y = 0.0f;
  bool in_image = true;

  if (keep_ratio) {
    const float scale = fminf(
      static_cast<float>(dst_height) / static_cast<float>(src_height),
      static_cast<float>(dst_width) / static_cast<float>(src_width));
    const int resized_width = static_cast<int>(static_cast<float>(src_width) * scale);
    const int resized_height = static_cast<int>(static_cast<float>(src_height) * scale);

    // Point4 CPU preprocess places the resized image at the top-left corner.
    if (dst_x >= resized_width || dst_y >= resized_height) {
      in_image = false;
    } else {
      const float resize_scale_x =
        static_cast<float>(resized_width) / static_cast<float>(src_width);
      const float resize_scale_y =
        static_cast<float>(resized_height) / static_cast<float>(src_height);
      src_x = (static_cast<float>(dst_x) + 0.5f) / resize_scale_x - 0.5f;
      src_y = (static_cast<float>(dst_y) + 0.5f) / resize_scale_y - 0.5f;
    }
  } else {
    const float scale_x = static_cast<float>(dst_width) / static_cast<float>(src_width);
    const float scale_y = static_cast<float>(dst_height) / static_cast<float>(src_height);
    src_x = (static_cast<float>(dst_x) + 0.5f) / scale_x - 0.5f;
    src_y = (static_cast<float>(dst_y) + 0.5f) / scale_y - 0.5f;
  }

  float rgb[3] = {0.0f, 0.0f, 0.0f};
  if (in_image) {
    // Source is BGR HWC. Point4 CPU preprocess writes RGB and normalizes to [0, 1].
    rgb[0] = read_bilinear_channel_point4(src, src_width, src_height, src_x, src_y, 2) / 255.0f;
    rgb[1] = read_bilinear_channel_point4(src, src_width, src_height, src_x, src_y, 1) / 255.0f;
    rgb[2] = read_bilinear_channel_point4(src, src_width, src_height, src_x, src_y, 0) / 255.0f;
  }

  const int pixel_idx = dst_y * dst_width + dst_x;
  const int hw = dst_width * dst_height;
  if (input_nhwc) {
    const int base = pixel_idx * 3;
    write_point4_value(dst, base + 0, rgb[0], output_fp16);
    write_point4_value(dst, base + 1, rgb[1], output_fp16);
    write_point4_value(dst, base + 2, rgb[2], output_fp16);
  } else {
    write_point4_value(dst, 0 * hw + pixel_idx, rgb[0], output_fp16);
    write_point4_value(dst, 1 * hw + pixel_idx, rgb[1], output_fp16);
    write_point4_value(dst, 2 * hw + pixel_idx, rgb[2], output_fp16);
  }
}

}  // namespace

void point4_preprocess_cuda(
  const unsigned char * src_device, int src_width, int src_height, void * dst_device,
  int dst_width, int dst_height, bool keep_ratio, bool input_nhwc, bool output_fp16,
  cudaStream_t stream)
{
  if (
    src_device == nullptr || dst_device == nullptr || src_width <= 0 || src_height <= 0 ||
    dst_width <= 0 || dst_height <= 0) {
    return;
  }

  dim3 block(16, 16);
  dim3 grid((dst_width + block.x - 1) / block.x, (dst_height + block.y - 1) / block.y);
  point4_preprocess_kernel<<<grid, block, 0, stream>>>(
    src_device, src_width, src_height, dst_device, dst_width, dst_height, keep_ratio, input_nhwc,
    output_fp16);
}

}  // namespace auto_aim
