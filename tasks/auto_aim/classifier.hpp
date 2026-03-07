#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

// Classifier需要OpenVINO支持
#ifdef ENABLE_OPENVINO
#include <openvino/openvino.hpp>

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);

  void classify(Armor & armor);

  void ovclassify(Armor & armor);

private:
  cv::dnn::Net net_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;
};

}  // namespace auto_aim

#endif  // ENABLE_OPENVINO

#endif  // AUTO_AIM__CLASSIFIER_HPP