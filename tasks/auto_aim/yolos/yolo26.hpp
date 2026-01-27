#ifndef AUTO_AIM__YOLO26_HPP
#define AUTO_AIM__YOLO26_HPP

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

class YOLO26 : public YOLOBase
{
public:
  YOLO26(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  std::string device_, model_path_;
  std::string save_path_;
  bool debug_, use_roi_;

  const int class_num_ = 16;
  const float nms_threshold_ = 0.3;
  const float score_threshold_ = 0.2;
  double min_confidence_, binary_threshold_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;  // 同步模式只需1个InferRequest

  // 预分配的输入图像和tensor，避免每帧重新分配
  cv::Mat input_image_;       // 640x640 RGB图像
  ov::Tensor input_tensor_;   // 预绑定的tensor

  cv::Rect roi_;
  cv::Point2f offset_;

  Detector detector_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, int pad_x, int pad_y, cv::Mat & output,
                         const cv::Mat & bgr_img, const cv::Mat & tmp_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  void sort_keypoints(std::vector<cv::Point2f> & keypoints);
};

}  // namespace auto_aim

#endif  //AUTO_AIM__YOLO26_HPP