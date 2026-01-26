#include "tracker.hpp"

#include <vector>
#include <yaml-cpp/yaml.h>

#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// ROS2 headers (仅在 ROS2 可用时编译)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_ && target_->diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  // 收敛效果检测：
  if (
    target_ &&
    std::accumulate(
      target_->ekf().recent_nis_failures.begin(), target_->ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_->ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return {};
  }

  if (state_ == "lost" || !target_) return {};

  std::list<Target> targets = {*target_};
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && target_ && armors.front().priority < target_->priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() && target_ &&
    temp_target.armors.front().priority < target_->priority && target_->convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_ && target_->diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost" || !target_) return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {*target_};
  
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
    temp_lost_count_ = 0;  // 重置临时丢失计数
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      temp_lost_count_ = 0;  // 重置临时丢失计数
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      // 前哨站在detecting阶段也允许较长时间丢失（最多10帧）
      // 因为旋转时可能某些角度检测不到装甲板，或被遮挡
      temp_lost_count_++;
      int max_detecting_lost = (target_ && target_->name == ArmorName::outpost) ? 10 : 0;
      
      if (temp_lost_count_ > max_detecting_lost) {
        detect_count_ = 0;
        state_ = "lost";
      }
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_ && target_->name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = std::make_shared<Target>(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    //  创建 OutpostTarget（13维状态）
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0, 100, 100}};  // 13维
    std::vector<double> armor_heights = {0.0, 0.0, 0.0};  // 初始高度差，会由EKF自动估计
    target_ = std::make_shared<OutpostTarget>(armor, t, 0.2765, 3, P0_dig, armor_heights);
    tools::logger()->info("✅ Created OutpostTarget (13D state)");
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = std::make_shared<Target>(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = std::make_shared<Target>(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (!target_) return false;
  
  target_->predict(t);

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_->name || armor.type != target_->armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (
      armor.name != target_->name || armor.type != target_->armor_type
      //  || armor.center.x != min_x
    )
      continue;

    solver_.solve(armor);

    target_->update(armor);
  }

  return true;
}

#ifdef AMENT_CMAKE_FOUND
void Tracker::set_ros2_node(std::shared_ptr<rclcpp::Node> node)
{
  ros_node_ = node;
  if (ros_node_) {
    // 创建 publisher 并存储为 void*（类型擦除）
    auto pub = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "armor_markers", 10);
    marker_pub_ = std::static_pointer_cast<void>(pub);
    tools::logger()->info("[Tracker] ROS2 marker publisher initialized on topic: armor_markers");
  }
}

void Tracker::publish_markers(
  const std::list<Target> & targets,
  const rclcpp::Time & timestamp,
  const Eigen::Vector4d & aim_xyza,
  bool has_aim_point)
{
  if (!ros_node_ || !marker_pub_) {
    return;  // ROS2 未初始化，直接返回
  }

  if (targets.empty()) {
    return;  // 没有目标，不发布
  }

  // 将 void* 转换回正确的类型
  auto pub = std::static_pointer_cast<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>(
    marker_pub_);

  visualization_msgs::msg::MarkerArray marker_array;

  // 遍历所有目标
  for (const auto & target : targets) {
    // 获取所有装甲板位姿
    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();

    // 为每个装甲板创建一个 marker
    int marker_id = 0;
    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";  // 根据你的坐标系修改
      marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
      marker.ns = "armor_plates";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;  // 改为球体
      marker.action = visualization_msgs::msg::Marker::ADD;

      // 设置位置 (x, y, z 在世界坐标系中)
      marker.pose.position.x = xyza[0];
      marker.pose.position.y = xyza[1];
      marker.pose.position.z = xyza[2];

      // 设置姿态 (从 yaw 角度转换为四元数)
      double yaw = xyza[3];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = std::sin(yaw / 2.0);
      marker.pose.orientation.w = std::cos(yaw / 2.0);

      // 设置尺寸 - 球体（直径）
      double diameter;
      if (target.armor_type == ArmorType::small) {
        diameter = 0.08;  // 小装甲板：80mm 直径
      } else {
        diameter = 0.12;  // 大装甲板：120mm 直径
      }
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = diameter;

      // 设置颜色 (绿色，半透明)
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.7;

      marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

      marker_array.markers.push_back(marker);
    }

    // 如果有瞄准点，也创建一个 marker（红色球体）
    if (has_aim_point) {
      visualization_msgs::msg::Marker aim_marker;
      aim_marker.header.frame_id = "world";
      aim_marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
      aim_marker.ns = "aim_point";
      aim_marker.id = 1000;
      aim_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 球体
      aim_marker.action = visualization_msgs::msg::Marker::ADD;

      aim_marker.pose.position.x = aim_xyza[0];
      aim_marker.pose.position.y = aim_xyza[1];
      aim_marker.pose.position.z = aim_xyza[2];

      double yaw = aim_xyza[3];
      aim_marker.pose.orientation.x = 0.0;
      aim_marker.pose.orientation.y = 0.0;
      aim_marker.pose.orientation.z = std::sin(yaw / 2.0);
      aim_marker.pose.orientation.w = std::cos(yaw / 2.0);

      // 设置尺寸 - 立着的长方体
      if (target.armor_type == ArmorType::small) {
        aim_marker.scale.x = 0.08;  // 宽度（水平方向）
        aim_marker.scale.y = 0.08;  // 厚度（很薄）
        aim_marker.scale.z = 0.08;  // 高度（竖直方向）
      } else {
        aim_marker.scale.x = 0.08;  // 宽度（水平方向）
        aim_marker.scale.y = 0.08;  // 厚度（很薄）
        aim_marker.scale.z = 0.08;  // 高度（竖直方向）
      }

      // 红色
      aim_marker.color.r = 1.0;
      aim_marker.color.g = 0.0;
      aim_marker.color.b = 0.0;
      aim_marker.color.a = 0.9;

      aim_marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

      marker_array.markers.push_back(aim_marker);
    }

    // 🆕 添加敌车中心的蓝色球体
    visualization_msgs::msg::Marker center_marker;
    center_marker.header.frame_id = "world";
    center_marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
    center_marker.ns = "vehicle_center";
    center_marker.id = 2000;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;

    // 从 EKF 状态获取敌车中心位置
    Eigen::VectorXd ekf_x = target.ekf_x();
    center_marker.pose.position.x = ekf_x[0];  // 中心 X
    center_marker.pose.position.y = ekf_x[2];  // 中心 Y
    center_marker.pose.position.z = ekf_x[4];  // 中心 Z

    // 姿态（可选，球体不受姿态影响）
    center_marker.pose.orientation.x = 0.0;
    center_marker.pose.orientation.y = 0.0;
    center_marker.pose.orientation.z = 0.0;
    center_marker.pose.orientation.w = 1.0;

    double center_diameter = 0.08;  // 80mm 直径
    center_marker.scale.x = center_diameter;
    center_marker.scale.y = center_diameter;
    center_marker.scale.z = center_diameter;

    // 蓝色
    center_marker.color.r = 0.0;
    center_marker.color.g = 0.0;
    center_marker.color.b = 1.0;
    center_marker.color.a = 0.8;

    center_marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

    marker_array.markers.push_back(center_marker);
  }

  // 发布 marker array
  pub->publish(marker_array);
}
#endif

}  // namespace auto_aim