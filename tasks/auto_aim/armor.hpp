#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
enum Color
{
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType
{
  big,
  small
};

const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName
{
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority
{
  first = 1,
  second,
  third,
  forth,
  fifth
};

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},       
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},    
  {blue, three, big},        {red, three, big},        {extinguish, three, big}, 
  {blue, four, big},         {red, four, big},         {extinguish, four, big},  
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

// YOLO11 模型的类别映射 (16类)
// 0: blue_G, 1: blue_1, 2: blue_2, 3: blue_3, 4: blue_4, 5: blue_5, 6: blue_O, 7: blue_B
// 8: red_G,  9: red_1,  10: red_2, 11: red_3, 12: red_4, 13: red_5, 14: red_O, 15: red_B
const std::vector<std::tuple<Color, ArmorName, ArmorType>> yolo11_armor_properties = {
  {blue, sentry, small},   // 0: blue_G
  {blue, one, small},      // 1: blue_1
  {blue, two, small},      // 2: blue_2
  {blue, three, small},    // 3: blue_3
  {blue, four, small},     // 4: blue_4
  {blue, five, small},     // 5: blue_5
  {blue, outpost, small},  // 6: blue_O
  {blue, base, big},       // 7: blue_B
  {red, sentry, small},    // 8: red_G
  {red, one, small},       // 9: red_1
  {red, two, small},       // 10: red_2
  {red, three, small},     // 11: red_3
  {red, four, small},      // 12: red_4
  {red, five, small},      // 13: red_5
  {red, outpost, small},   // 14: red_O
  {red, base, big}         // 15: red_B
};

// YOLO26 模型的类别映射 (16类) 
// 0: red_1, 1: red_2, 2: red_3, 3: red_4, 4: red_5, 5: red_G, 6: red_O, 7: red_B
// 8: blue_1, 9: blue_2, 10: blue_3, 11: blue_4, 12: blue_5, 13: blue_G, 14: blue_O, 15: blue_B
const std::vector<std::tuple<Color, ArmorName, ArmorType>> yolo26_armor_properties = {
  {red, one, small},       // 0: red_1
  {red, two, small},       // 1: red_2
  {red, three, small},     // 2: red_3
  {red, four, small},      // 3: red_4
  {red, five, small},      // 4: red_5
  {red, sentry, small},    // 5: red_6 (哨兵)
  {red, outpost, small},   // 6: red_7 (前哨站)
  {red, base, big},        // 7: red_8 (基地)
  {blue, one, small},      // 8: blue_1
  {blue, two, small},      // 9: blue_2
  {blue, three, small},    // 10: blue_3
  {blue, four, small},     // 11: blue_4
  {blue, five, small},     // 12: blue_5
  {blue, sentry, small},   // 13: blue_6 (哨兵)
  {blue, outpost, small},  // 14: blue_7 (前哨站)
  {blue, base, big}        // 15: blue_8 (基地)
};

// YOLO 版本枚举（用于选择类别映射表）
enum class YOLOVersion
{
  YOLO11,
  YOLO26
};

struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

struct Armor
{
  Color color;
  Lightbar left, right;     //used to be const
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio;              // 两灯条的中点连线与长灯条的长度之比
  double side_ratio;         // 长灯条与短灯条的长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2的差值

  ArmorType type;
  ArmorName name;
  ArmorPriority priority;
  int class_id;
  cv::Rect box;
  cv::Mat pattern;
  double confidence;
  bool duplicated;

  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d xyz_in_world;   // 单位：m
  Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
  Eigen::Vector3d ypr_in_world;   // 单位：rad
  Eigen::Vector3d ypd_in_world;   // 球坐标系

  double yaw_raw;  // rad

  Armor(const Lightbar & left, const Lightbar & right);
  // YOLO11/YOLO26 构造函数（不带ROI）
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    YOLOVersion version = YOLOVersion::YOLO11);
  // YOLO11/YOLO26 构造函数（带ROI）
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset, YOLOVersion version = YOLOVersion::YOLO11);
  // YOLOV5 构造函数（不带ROI）
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  // YOLOV5 构造函数（带ROI）
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP