#ifndef AUTO_AIM__YOLO26_TRT_HPP
#define AUTO_AIM__YOLO26_TRT_HPP

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/yolos/tensorrt_logger.hpp"

namespace auto_aim
{

class YOLO26_TRT : public YOLOBase
{
public:
  YOLO26_TRT(const std::string & config_path, bool debug);
  ~YOLO26_TRT();

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string device_;
  std::string engine_path_;
  std::string onnx_path_;
  std::string save_path_;
  bool debug_;
  bool use_roi_;
  bool swap_rb_channels_ = false;
  bool use_cuda_preprocess_ = false;

  const int class_num_ = 16;
  float nms_threshold_ = 0.3f;
  float score_threshold_ = 0.2f;
  double min_confidence_;
  double binary_threshold_;

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime, void(*)(nvinfer1::IRuntime*)> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)> context_;

  cudaStream_t stream_;
  void * buffers_[2] = {nullptr, nullptr};
  unsigned char * gpu_img_buffer_ = nullptr;
  size_t gpu_img_buffer_bytes_ = 0;

  nvinfer1::DataType input_dtype_;
  nvinfer1::DataType output_dtype_;
  std::vector<float> input_host_float_;
  std::vector<__half> input_host_half_;
  std::vector<float> output_host_float_;
  std::vector<__half> output_host_half_;

  size_t input_size_bytes_ = 0;
  size_t output_size_bytes_ = 0;

  int input_w_ = 640;
  int input_h_ = 640;
  int output_rows_ = 0;
  int output_stride_ = 14;
  bool output_channel_first_ = false;

  cv::Mat input_image_;
  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;
  Detector detector_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;

  std::list<Armor> parse(
    float scale, int pad_x, int pad_y, const float * output_data, int num_rows, int stride,
    const cv::Mat & bgr_img, const cv::Mat & tmp_img, int frame_count);

  void loadEngine(const std::string & engine_file);
  void buildEngineFromONNX(const std::string & onnx_file);
  void serializeEngine(const std::string & engine_file);
  void prepareOutputLayoutFromEngine(const char * output_name);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO26_TRT_HPP
