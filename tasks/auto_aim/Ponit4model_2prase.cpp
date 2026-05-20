#include "Point4model.hpp"

#ifdef ENABLE_TENSORRT

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/dnn.hpp>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
constexpr int kPoint4Stride = 22;
constexpr int kPoint4ObjectnessIndex = 8;
constexpr int kPoint4ColorOffset = 9;
constexpr int kPoint4ColorCount = 4;
constexpr int kPoint4NumberOffset = 13;
constexpr int kPoint4NumberCount = 9;

int argmax_n(const float * data, int n)
{
  int best = 0;
  for (int i = 1; i < n; ++i) {
    if (data[i] > data[best]) best = i;
  }
  return best;
}

bool all_points_finite(const std::vector<cv::Point2f> & points)
{
  return std::all_of(points.begin(), points.end(), [](const cv::Point2f & p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
  });
}

void sort_armor_points(std::vector<cv::Point2f> & points)
{
  if (points.size() != 4) return;

  std::sort(points.begin(), points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  std::vector<cv::Point2f> top{points[0], points[1]};
  std::vector<cv::Point2f> bottom{points[2], points[3]};

  std::sort(top.begin(), top.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
  std::sort(bottom.begin(), bottom.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  points[0] = top[0];
  points[1] = top[1];
  points[2] = bottom[1];
  points[3] = bottom[0];
}

cv::Rect bounding_rect_from_points(
  const std::vector<cv::Point2f> & points, const cv::Size & image_size)
{
  if (points.empty()) return {};

  float min_x = points.front().x;
  float max_x = points.front().x;
  float min_y = points.front().y;
  float max_y = points.front().y;
  for (const auto & p : points) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  const int left = std::clamp(static_cast<int>(std::floor(min_x)), 0, image_size.width - 1);
  const int top = std::clamp(static_cast<int>(std::floor(min_y)), 0, image_size.height - 1);
  const int right = std::clamp(static_cast<int>(std::ceil(max_x)), 0, image_size.width);
  const int bottom = std::clamp(static_cast<int>(std::ceil(max_y)), 0, image_size.height);
  if (right <= left || bottom <= top) return {};

  return {left, top, right - left, bottom - top};
}

std::vector<cv::Point> to_cv_points(
  const std::vector<cv::Point2f> & points, const cv::Point2f & offset = {})
{
  std::vector<cv::Point> cv_points;
  cv_points.reserve(points.size());
  for (const auto & p : points) {
    cv_points.emplace_back(cvRound(p.x + offset.x), cvRound(p.y + offset.y));
  }
  return cv_points;
}

void draw_point4_polygon(
  cv::Mat & canvas, const std::vector<cv::Point2f> & points, const cv::Scalar & color,
  int thickness, const cv::Point2f & offset = {})
{
  if (points.size() != 4) return;

  const auto cv_points = to_cv_points(points, offset);
  cv::polylines(canvas, cv_points, true, color, thickness, cv::LINE_AA);
  for (int i = 0; i < static_cast<int>(cv_points.size()); ++i) {
    cv::circle(canvas, cv_points[i], 3, color, -1, cv::LINE_AA);
    cv::putText(
      canvas, std::to_string(i), cv_points[i] + cv::Point(3, -3), cv::FONT_HERSHEY_SIMPLEX, 0.38,
      color, 1, cv::LINE_AA);
  }
}

template <typename DetectionT>
void show_2parse_debug_window(
  const cv::Mat & raw_img, const std::vector<DetectionT> & detections, const std::vector<int> & indices,
  const std::list<Armor> & armors, bool use_roi, const cv::Rect & roi, const cv::Point2f & offset)
{
  if (raw_img.empty()) return;

  cv::Mat debug_img = raw_img.clone();
  const cv::Point2f draw_offset = use_roi ? offset : cv::Point2f{};

  if (use_roi) {
    cv::rectangle(debug_img, roi & cv::Rect(0, 0, raw_img.cols, raw_img.rows), cv::Scalar(255, 255, 0), 1);
  }

  for (const auto & det : detections) {
    draw_point4_polygon(debug_img, det.points, cv::Scalar(0, 165, 255), 1, draw_offset);
    cv::Rect box = det.box;
    if (use_roi) box += cv::Point(cvRound(offset.x), cvRound(offset.y));
    cv::rectangle(debug_img, box, cv::Scalar(0, 165, 255), 1);
  }

  for (int idx : indices) {
    if (idx < 0 || idx >= static_cast<int>(detections.size())) continue;
    const auto & det = detections[idx];
    draw_point4_polygon(debug_img, det.points, cv::Scalar(255, 255, 0), 2, draw_offset);
  }

  for (const auto & armor : armors) {
    draw_point4_polygon(debug_img, armor.points, cv::Scalar(0, 255, 0), 2);
    cv::putText(
      debug_img,
      fmt::format("{:.2f} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name]),
      armor.center + cv::Point2f(4, -4), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2,
      cv::LINE_AA);
  }

  cv::putText(
    debug_img,
    fmt::format(
      "2parse raw={} nms={} armor={}  orange:raw cyan:nms green:armor", detections.size(),
      indices.size(), armors.size()),
    {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(
    debug_img,
    fmt::format(
      "2parse raw={} nms={} armor={}  orange:raw cyan:nms green:armor", detections.size(),
      indices.size(), armors.size()),
    {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

  cv::imshow("Point4 2parse debug", debug_img);
}

}  // namespace

std::list<Armor> Point4Model2ParseTRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(
    output.ptr<float>(0), output.rows, output.cols, static_cast<float>(scale),
    static_cast<float>(scale), bgr_img, frame_count);
}

std::list<Armor> Point4Model2ParseTRT::parse(
  const float * output_data, int rows, int stride, float x_scale, float y_scale,
  const cv::Mat & raw_img, int frame_count)
{
  if (!output_data || rows <= 0 || stride < kPoint4Stride || raw_img.empty()) {
    return {};
  }

  std::vector<Detection> detections;
  detections.reserve(std::max(32, rows / 8));
  std::vector<cv::Rect> boxes;
  std::vector<float> nms_scores;

  const cv::Size parse_image_size = use_roi_ ? roi_.size() : raw_img.size();
  if (parse_image_size.width <= 0 || parse_image_size.height <= 0) return {};

  for (int r = 0; r < rows; ++r) {
    const float * row = output_data + r * stride;
    const float object_conf = static_cast<float>(sigmoid(row[kPoint4ObjectnessIndex]));
    if (object_conf < score_threshold_) continue;

    int color_id = argmax_n(row + kPoint4ColorOffset, kPoint4ColorCount);
    if (color_id == Color::extinguish || color_id == Color::purple) continue;
    if (swap_color_id_) color_id = 1 - color_id;
    if (filter_enemy_color_ && detect_color_ >= 0 && color_id != detect_color_) continue;

    const int num_id = argmax_n(row + kPoint4NumberOffset, kPoint4NumberCount);
    const float num_score = row[kPoint4NumberOffset + num_id];

    // 模型角点顺序统一成 Armor 约定：左上、右上、右下、左下。
    std::vector<cv::Point2f> points;
    points.reserve(4);
    points.emplace_back(row[0] / x_scale, row[1] / y_scale);
    points.emplace_back(row[6] / x_scale, row[7] / y_scale);
    points.emplace_back(row[4] / x_scale, row[5] / y_scale);
    points.emplace_back(row[2] / x_scale, row[3] / y_scale);
    if (!all_points_finite(points)) continue;

    sort_armor_points(points);
    const cv::Rect box = bounding_rect_from_points(points, parse_image_size);
    if (box.empty()) continue;
    if (cv::contourArea(points) < 4.0) continue;

    const float nms_score = std::max(object_conf, num_score);
    detections.push_back({color_id, num_id, object_conf, nms_score, box, points});
    boxes.push_back(box);
    nms_scores.push_back(nms_score);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, nms_scores, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (int i : indices) {
    const auto & det = detections[i];
    if (use_roi_) {
      armors.emplace_back(
        det.color_id, det.num_id, det.object_conf, det.box, det.points, offset_);
    } else {
      armors.emplace_back(det.color_id, det.num_id, det.object_conf, det.box, det.points);
    }
  }

  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it) || !check_type(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (use_traditional_) {
      detector_.detect(*it, raw_img);
    }
    it->center_norm = get_center_norm(raw_img, it->center);
    ++it;
  }

  if (debug_) {
    show_2parse_debug_window(raw_img, detections, indices, armors, use_roi_, roi_, offset_);
    draw_detections(raw_img, armors, frame_count);
  }

  return armors;
}

}  // namespace auto_aim

#endif  // ENABLE_TENSORRT
