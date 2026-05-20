#ifndef AUTO_AIM__POINT4MODEL_HPP
#define AUTO_AIM__POINT4MODEL_HPP

#ifdef ENABLE_TENSORRT

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

class Point4ModelTRT : public YOLOBase
{
public:
  Point4ModelTRT(const std::string & config_path, bool debug = true);
  ~Point4ModelTRT();

  std::list<Armor> detect(const cv::Mat & raw_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

protected:
  struct Detection
  {
    int color_id;
    int num_id;
    float object_conf;
    float nms_score;
    cv::Rect box;
    std::vector<cv::Point2f> points;
  };

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime, void (*)(nvinfer1::IRuntime *)> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, void (*)(nvinfer1::ICudaEngine *)> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, void (*)(nvinfer1::IExecutionContext *)> context_;

  cudaStream_t stream_ = nullptr;
  cudaEvent_t infer_start_event_ = nullptr;
  cudaEvent_t infer_end_event_ = nullptr;
  void * buffers_[2] = {nullptr, nullptr};
  unsigned char * gpu_img_buffer_ = nullptr;
  size_t gpu_img_buffer_bytes_ = 0;

  std::string engine_path_;
  std::string onnx_path_;
  std::string device_;
  bool debug_ = true;
  bool use_roi_ = false;
  bool use_traditional_ = false;
  bool keep_ratio_ = true;
  bool use_cuda_preprocess_ = false;
  bool filter_enemy_color_ = false;
  bool swap_color_id_ = false;
  int detect_color_ = -1;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;
  Detector detector_;

  nvinfer1::DataType input_dtype_;
  nvinfer1::DataType output_dtype_;
  bool input_nhwc_ = false;
  bool output_channel_first_ = false;
  int input_w_ = 640;
  int input_h_ = 640;
  int output_rows_ = 0;
  int output_stride_ = 22;

  size_t input_size_bytes_ = 0;
  size_t output_size_bytes_ = 0;
  std::vector<float> input_host_float_;
  std::vector<__half> input_host_half_;
  std::vector<float> output_host_float_;
  std::vector<__half> output_host_half_;

  float score_threshold_ = 0.65f;
  float nms_threshold_ = 0.45f;
  double min_confidence_ = 0.0;
  float last_gpu_infer_ms_ = 0.0f;

  void load_engine(const std::string & engine_file);
  void build_engine_from_onnx(const std::string & onnx_file);
  void serialize_engine(const std::string & engine_file);
  void prepare_tensor_layouts(const char * input_name, const char * output_name);
  void preprocess(
    const cv::Mat & bgr_img, float & x_scale, float & y_scale, int & resized_w, int & resized_h);
  virtual std::list<Armor> parse(
    const float * output_data, int rows, int stride, float x_scale, float y_scale,
    const cv::Mat & raw_img, int frame_count);

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;

  static double sigmoid(double x);
  static size_t dtype_size(nvinfer1::DataType dtype);
};

class Point4Model2ParseTRT : public Point4ModelTRT
{
public:
  using Point4ModelTRT::Point4ModelTRT;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

protected:
  std::list<Armor> parse(
    const float * output_data, int rows, int stride, float x_scale, float y_scale,
    const cv::Mat & raw_img, int frame_count) override;
};

}  // namespace auto_aim

#endif  // ENABLE_TENSORRT

#endif  // AUTO_AIM__POINT4MODEL_HPP
