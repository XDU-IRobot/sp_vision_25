#ifndef AUTO_AIM__YOLO11_HPP
#define AUTO_AIM__YOLO11_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class YOLO11 : public YOLOBase
{
public:
  YOLO11(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_;
  bool use_async_inference_;  // 是否使用异步推理
  bool use_traditional_;       // 是否使用传统方法微调角点

  const int class_num_ = 16;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.7;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  // 同步推理优化：预分配输入buffer并预绑定Tensor，避免每帧构造Tensor/set_input_tensor
  cv::Mat input_image_;
  ov::Tensor input_tensor_;

  // 异步推理：使用2个InferRequest形成流水线
  ov::InferRequest infer_request_current_;  // 当前帧推理请求
  ov::InferRequest infer_request_next_;     // 下一帧推理请求
  bool first_frame_ = true;                 // 是否为第一帧

  // 存储上一帧的预处理结果（用于异步流水线）
  cv::Mat prev_preprocessed_img_;
  cv::Mat prev_raw_img_;  // 保存上一帧的原始图像（用于parse）
  float prev_scale_ = 1.0f;
  int prev_pad_x_ = 0;
  int prev_pad_y_ = 0;
  int prev_frame_count_ = 0;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  Detector detector_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(float scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO11_HPP