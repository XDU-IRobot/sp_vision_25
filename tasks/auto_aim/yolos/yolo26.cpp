#include "yolo26.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
YOLO26::YOLO26(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["yolo26_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  binary_threshold_ = yaml["threshold"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);

  // TODO: ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLO26::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  cv::Mat bgr_img;
  tmp_img_ = raw_img;
  if (use_roi_) {
    if (roi_.width == -1) {  // -1 表示该维度不裁切
      roi_.width = raw_img.cols;
    }
    if (roi_.height == -1) {  // -1 表示该维度不裁切
      roi_.height = raw_img.rows;
    }
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  int orig_w = bgr_img.cols;
  int orig_h = bgr_img.rows;

  // Letterbox resize 到 640x640（居中padding）
  float scale = std::min(640.0f / orig_w, 640.0f / orig_h);
  int new_w = static_cast<int>(orig_w * scale);
  int new_h = static_cast<int>(orig_h * scale);
  int pad_x = (640 - new_w) / 2;  // 居中padding
  int pad_y = (640 - new_h) / 2;

  // 创建640x640的黑色画布，将缩放后的图像居中放置
  cv::Mat input(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::resize(bgr_img, input(cv::Rect(pad_x, pad_y, new_w, new_h)),
             cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);
                                                                           
  /// infer
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());

  return parse(scale, pad_x, pad_y, output, bgr_img, frame_count);  // 传入 bgr_img（ROI裁剪后）
}
//修改了这里
std::list<Armor> YOLO26::parse(
  double scale, int pad_x, int pad_y, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // 调试：打印原始输出形状
  tools::logger()->info("[YOLO26] Output shape: rows={}, cols={} (should be [N_detections, 18])",
                        output.rows, output.cols);
  tools::logger()->info("[YOLO26] bgr_img size: {}x{}, scale={}, pad_x={}, pad_y={}",
                        bgr_img.cols, bgr_img.rows, scale, pad_x, pad_y);
  tools::logger()->info("[YOLO26] use_roi_={}, offset_=({}, {})", use_roi_, offset_.x, offset_.y);

  std::vector<int> ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;

  int total_detections = 0;
  int parsed_count = 0;
  float max_conf = 0.0f;
  float min_conf = 999.0f;

  for (int r = 0; r < output.rows; r++) {
    // 格式：[xyxy(4), confidence(1), class_id(1), keypoints(12)] = 18维
    float x1_raw = output.at<float>(r, 0);
    float y1_raw = output.at<float>(r, 1);
    float x2_raw = output.at<float>(r, 2);
    float y2_raw = output.at<float>(r, 3);
    float conf = output.at<float>(r, 4);
    int cls = static_cast<int>(output.at<float>(r, 5));

    total_detections++;

    // 统计置信度范围
    max_conf = std::max(max_conf, conf);
    min_conf = std::min(min_conf, conf);

    // 调试：打印前5个检测的原始数据（不管是否通过阈值）
    if (total_detections <= 5) {
      tools::logger()->info("[YOLO26] Raw detection {}: x1={:.1f}, y1={:.1f}, x2={:.1f}, y2={:.1f}, conf={:.6f}, cls={}, threshold={:.3f}",
                            total_detections, x1_raw, y1_raw, x2_raw, y2_raw, conf, cls, score_threshold_);
    }

    // ---------- 1. 置信度过滤 ----------
    if (conf < score_threshold_) {
      continue;
    }

    parsed_count++;

    // 调试：打印前几个检测的原始数据
    if (parsed_count <= 3) {
      tools::logger()->info("[YOLO26] Detection {}: x1={:.1f}, y1={:.1f}, x2={:.1f}, y2={:.1f}, conf={:.3f}, cls={}",
                            parsed_count, x1_raw, y1_raw, x2_raw, y2_raw, conf, cls);
    }

    // ---------- 2. bbox (xyxy) - 减去padding偏移并限制范围 ----------
    int x1 = static_cast<int>((x1_raw - pad_x) / scale);
    int y1 = static_cast<int>((y1_raw - pad_y) / scale);
    int x2 = static_cast<int>((x2_raw - pad_x) / scale);
    int y2 = static_cast<int>((y2_raw - pad_y) / scale);

    // 限制坐标在图像范围内
    x1 = std::clamp(x1, 0, bgr_img.cols);
    x2 = std::clamp(x2, 0, bgr_img.cols);
    y1 = std::clamp(y1, 0, bgr_img.rows);
    y2 = std::clamp(y2, 0, bgr_img.rows);

    int left   = x1;
    int top    = y1;
    int width  = std::max(0, x2 - x1);
    int height = std::max(0, y2 - y1);

    // 调试：打印坐标转换
    if (parsed_count <= 3) {
      tools::logger()->info("[YOLO26] After transform: left={}, top={}, width={}, height={}",
                            left, top, width, height);
    }

    // ---------- 3. keypoints (4×3 格式：x, y, visibility) ----------
    std::vector<cv::Point2f> armor_key_points;
    for (int i = 0; i < 4; i++) {
      float kx_raw = output.at<float>(r, 6 + i * 3 + 0);
      float ky_raw = output.at<float>(r, 6 + i * 3 + 1);
      // float visibility = output.at<float>(r, 6 + i * 3 + 2);  // 忽略

      float kx = (kx_raw - pad_x) / scale;
      float ky = (ky_raw - pad_y) / scale;
      armor_key_points.emplace_back(kx, ky);
    }

    ids.emplace_back(cls);
    confidences.emplace_back(conf);
    boxes.emplace_back(left, top, width, height);
    armors_key_points.emplace_back(armor_key_points);
  }

  // ---------- 5. NMS (非极大值抑制) ----------
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  tools::logger()->info("[YOLO26] Total detections: {}, Max conf: {:.6f}, Min conf: {:.6f}",
                        total_detections, max_conf, min_conf);
  tools::logger()->info("[YOLO26] Passed threshold ({}): {} detections, After NMS: {} detections",
                        score_threshold_, boxes.size(), indices.size());

  // ---------- 6. 生成 Armor ----------
  std::list<Armor> armors;
  for (int i : indices) {
    sort_keypoints(armors_key_points[i]);

    tools::logger()->info("[YOLO26] Creating armor {}: id={}, conf={:.3f}, box=({},{},{}x{})",
                          i, ids[i], confidences[i],
                          boxes[i].x, boxes[i].y, boxes[i].width, boxes[i].height);

    if (use_roi_) {
      tools::logger()->info("[YOLO26] ROI mode: adding offset ({}, {})", offset_.x, offset_.y);
      armors.emplace_back(
        ids[i],
        confidences[i],
        boxes[i],
        armors_key_points[i],
        offset_,
        YOLOVersion::YOLO26
      );
    } else {
      armors.emplace_back(
        ids[i],
        confidences[i],
        boxes[i],
        armors_key_points[i],
        YOLOVersion::YOLO26
      );
    }
  }

  tools::logger()->info("[YOLO26] Created {} armors", armors.size());

  // ---------- 7. 过滤 ----------
  int filtered_count = 0;
  for (auto it = armors.begin(); it != armors.end();) {
    bool name_ok = check_name(*it);
    bool type_ok = check_type(*it);

    if (!name_ok || !type_ok) {
      tools::logger()->info("[YOLO26] Filtered out armor: name={}, type={}, name_ok={}, type_ok={}",
                            static_cast<int>(it->name), static_cast<int>(it->type), name_ok, type_ok);
      filtered_count++;
      it = armors.erase(it);
      continue;
    }
    // 注意：此时armor.center已经是全局坐标（相对于raw_img/tmp_img_），所以归一化也要用tmp_img_的尺寸
    it->center_norm = get_center_norm(tmp_img_, it->center);
    ++it;
  }

  tools::logger()->info("[YOLO26] After filtering: {} armors remain, {} filtered out",
                        armors.size(), filtered_count);
  tools::logger()->info("[YOLO26] debug_={}, will {} call draw_detections",
                        debug_, debug_ ? "YES" : "NO");

  if (debug_) {
    tools::logger()->info("[YOLO26] Calling draw_detections with {} armors", armors.size());
    // 当use_roi_=true时，armor坐标已经是全局坐标（相对于raw_img），所以要在tmp_img_上绘制
    // 当use_roi_=false时，tmp_img_就是bgr_img，效果相同
    draw_detections(tmp_img_, armors, frame_count);
  }

  return armors;
}

bool YOLO26::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLO26::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于神经网络的迭代
  // if (!name_ok) save(armor);

  return name_ok;
}

cv::Point2f YOLO26::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLO26::sort_keypoints(std::vector<cv::Point2f> & keypoints)
{
  if (keypoints.size() != 4) {
    std::cout << "beyond 4!!" << std::endl;
    return;
  }

  std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
  std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

  std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });

  std::sort(
    bottom_points.begin(), bottom_points.end(),
    [](const cv::Point2f & a, const cv::Point2f & b) { return a.x < b.x; });

  keypoints[0] = top_points[0];     // top-left
  keypoints[1] = top_points[1];     // top-right
  keypoints[2] = bottom_points[1];  // bottom-right
  keypoints[3] = bottom_points[0];  // bottom-left
}

void YOLO26::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  tools::logger()->info("[YOLO26] draw_detections called: img size={}x{}, armors count={}",
                        img.cols, img.rows, armors.size());

  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  int armor_idx = 0;
  for (const auto & armor : armors) {
    tools::logger()->info("[YOLO26] Drawing armor {}: points[0]=({:.1f},{:.1f}), points[1]=({:.1f},{:.1f}), points[2]=({:.1f},{:.1f}), points[3]=({:.1f},{:.1f})",
                          armor_idx,
                          armor.points[0].x, armor.points[0].y,
                          armor.points[1].x, armor.points[1].y,
                          armor.points[2].x, armor.points[2].y,
                          armor.points[3].x, armor.points[3].y);

    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
    armor_idx++;
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  tools::logger()->info("[YOLO26] Calling cv::imshow with detection image {}x{}", detection.cols, detection.rows);
  cv::imshow("detection", detection);
}

void YOLO26::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

std::list<Armor> YOLO26::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  // postprocess 假设没有 letterbox padding（外部调用场景）
  return parse(scale, 0, 0, output, bgr_img, frame_count);
}

}  // namespace auto_aim