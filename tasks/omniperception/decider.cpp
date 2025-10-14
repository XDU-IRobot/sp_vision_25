#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace omniperception {
Decider::Decider(const std::string &config_path)
    : detector_(config_path), count_(0) {
  // 使用更健壮的 YAML 读取，允许缺省并提供合理默认值
  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(config_path);
  } catch (const std::exception &e) {
    tools::logger()->warn(
        "[Omni/Decider] YAML load failed: {}. Using defaults.", e.what());
  }

  auto read_double = [&](const YAML::Node &n, const char *key,
                         double def) -> double {
    try {
      if (n && n[key])
        return n[key].as<double>();
    } catch (const std::exception &e) {
      tools::logger()->warn("[Omni/Decider] key '{}' bad double: {}. use {}",
                            key, e.what(), def);
    }
    return def;
  };
  auto read_int = [&](const YAML::Node &n, const char *key, int def) -> int {
    try {
      if (n && n[key])
        return n[key].as<int>();
    } catch (const std::exception &e) {
      tools::logger()->warn("[Omni/Decider] key '{}' bad int: {}. use {}", key,
                            e.what(), def);
    }
    return def;
  };
  auto read_str = [&](const YAML::Node &n, const char *key,
                      const std::string &def) -> std::string {
    try {
      if (n && n[key])
        return n[key].as<std::string>();
    } catch (const std::exception &e) {
      tools::logger()->warn("[Omni/Decider] key '{}' bad string: {}. use '{}'",
                            key, e.what(), def);
    }
    return def;
  };

  // 图像尺寸：优先读取 image_width/height；否则从 camera_settings
  // 中取；最后回退默认
  int default_w = 1920, default_h = 1080;
  if (yaml["camera_settings"]) {
    default_w = static_cast<int>(
        read_double(yaml["camera_settings"], "width", default_w));
    default_h = static_cast<int>(
        read_double(yaml["camera_settings"], "height", default_h));
  }
  img_width_ = static_cast<int>(read_double(yaml, "image_width", default_w));
  img_height_ = static_cast<int>(read_double(yaml, "image_height", default_h));

  // 视场角：缺省给出常用值；new_fov* 缺省跟随 fov*
  fov_h_ = read_double(yaml, "fov_h", 67.0);
  fov_v_ = read_double(yaml, "fov_v", 40.9);
  new_fov_h_ = read_double(yaml, "new_fov_h", fov_h_);
  new_fov_v_ = read_double(yaml, "new_fov_v", fov_v_);

  // 敌我颜色：缺省 blue/red 均可，这里默认 red
  auto color_str = read_str(yaml, "enemy_color", std::string("red"));
  enemy_color_ =
      (color_str == "red") ? auto_aim::Color::red : auto_aim::Color::blue;

  // 模式：缺省 1（与原项目约定）
  mode_ = read_int(yaml, "mode", MODE_ONE);

  tools::logger()->info("[Omni/Decider] init: img={}x{}, fov=({:.1f},{:.1f}) "
                        "new_fov=({:.1f},{:.1f}) color={} mode={}",
                        img_width_, img_height_, fov_h_, fov_v_, new_fov_h_,
                        new_fov_v_, color_str, mode_);
}

io::Command Decider::decide(auto_aim::YOLO &yolo,
                            const Eigen::Vector3d &gimbal_pos,
                            io::USBCamera &usbcam1, io::USBCamera &usbcam2,
                            io::Camera &back_camera) {
  Eigen::Vector2d delta_angle;
  io::USBCamera *cams[] = {&usbcam1, &usbcam2};

  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  auto armors = yolo.detect(usb_img);
  auto empty = armor_filter(armors);

  if (!empty) {
    if (count_ == 2) {
      delta_angle = this->delta_angle(armors, "back");
    } else {
      delta_angle = this->delta_angle(armors, cams[count_]->device_name);
    }

    tools::logger()->debug("[{} camera] delta yaw:{:.2f},target "
                           "pitch:{:.2f},armor number:{},armor name:{}",
                           (count_ == 2 ? "back" : cams[count_]->device_name),
                           delta_angle[0], delta_angle[1], armors.size(),
                           auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % 3;

    return io::Command{true, false,
                       tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
                       tools::limit_rad(delta_angle[1] / 57.3)};
  }

  count_ = (count_ + 1) % 3;
  // 如果没有找到目标，返回默认命令
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(auto_aim::YOLO &yolo,
                            const Eigen::Vector3d &gimbal_pos,
                            io::Camera &back_cammera) {
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  back_cammera.read(img, timestamp);
  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto delta_angle = this->delta_angle(armors, "back");
    tools::logger()->debug("[back camera] delta yaw:{:.2f},target "
                           "pitch:{:.2f},armor number:{},armor name:{}",
                           delta_angle[0], delta_angle[1], armors.size(),
                           auto_aim::ARMOR_NAMES[armors.front().name]);

    return io::Command{true, false,
                       tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
                       tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

io::Command
Decider::decide(const std::vector<DetectionResult> &detection_queue) {
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  DetectionResult dr = detection_queue.front();
  if (dr.armors.empty())
    return io::Command{false, false, 0, 0};
  tools::logger()->info("omniperceptron find {},delta yaw is {:.4f}",
                        auto_aim::ARMOR_NAMES[dr.armors.front().name],
                        dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};

Eigen::Vector2d Decider::delta_angle(const std::list<auto_aim::Armor> &armors,
                                     const std::string &camera) {
  Eigen::Vector2d delta_angle;
  if (camera == "left") {
    delta_angle[0] =
        62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else if (camera == "right") {
    delta_angle[0] =
        -62 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else {
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }
}

bool Decider::armor_filter(std::list<auto_aim::Armor> &armors) {
  if (armors.empty())
    return true;
  // 过滤非敌方装甲板
  armors.remove_if(
      [&](const auto_aim::Armor &a) { return a.color != enemy_color_; });

  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor &a) {
    return a.name == auto_aim::ArmorName::five;
  });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name ==
  // auto_aim::ArmorName::two; }); 不打前哨站
  armors.remove_if([&](const auto_aim::Armor &a) {
    return a.name == auto_aim::ArmorName::outpost;
  });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor &a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(),
                     a.name) != invincible_armor_.end();
  });

  return armors.empty();
}

void Decider::set_priority(std::list<auto_aim::Armor> &armors) {
  if (armors.empty())
    return;

  const PriorityMap &priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto &armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

void Decider::sort(std::vector<DetectionResult> &detection_queue) {
  if (detection_queue.empty())
    return;

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto &dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort([](const auto_aim::Armor &a, const auto_aim::Armor &b) {
      return a.priority < b.priority;
    });
  }

  // 根据优先级对 DetectionResult 进行排序
  std::sort(detection_queue.begin(), detection_queue.end(),
            [](const DetectionResult &a, const DetectionResult &b) {
              return a.armors.front().priority < b.armors.front().priority;
            });
}

Eigen::Vector4d
Decider::get_target_info(const std::list<auto_aim::Armor> &armors,
                         const std::list<auto_aim::Target> &targets) {
  if (armors.empty() || targets.empty())
    return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto &armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
                             static_cast<double>(armor.name) +
                                 1}; //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(
    const std::vector<int8_t> &invincible_enemy_ids) {
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty())
    return;

  for (const auto &id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

void Decider::get_auto_aim_target(std::list<auto_aim::Armor> &armors,
                                  const std::vector<int8_t> &auto_aim_target) {
  if (auto_aim_target.empty())
    return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto &target : auto_aim_target) {
    if (target <= 0 ||
        static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}",
                            int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}",
                          auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty())
    return;

  armors.remove_if([&](const auto_aim::Armor &a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(),
                     a.name) == auto_aim_targets.end();
  });
}

} // namespace omniperception