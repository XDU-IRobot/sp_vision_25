#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <memory>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

// 前向声明
namespace io {
class CBoard;
}

// ROS2 前向声明（避免强制依赖）
#ifdef AMENT_CMAKE_FOUND
namespace rclcpp {
class Node;
class Time;
}
#endif

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

  // 🆕 动态设置敌方颜色（用于根据robot_id实时切换）
  void set_enemy_color(Color color) { enemy_color_ = color; }

  // 🆕 获取当前敌方颜色
  Color get_enemy_color() const { return enemy_color_; }

  // 🆕 设置CBoard指针（用于自动从robot_id获取敌方颜色）
  void set_cboard(io::CBoard* cboard) { cboard_ = cboard; }

#ifdef AMENT_CMAKE_FOUND
  // 设置 ROS2 节点用于发布 marker（可选）
  void set_ros2_node(std::shared_ptr<rclcpp::Node> node);

  // 发布当前跟踪目标的 marker 到 rviz2
  // timestamp: 可选的时间戳，如果不提供则使用ros_node_->now()
  void publish_markers(
    const std::list<Target> & targets,
    const rclcpp::Time & timestamp,
    const Eigen::Vector4d & aim_xyza = Eigen::Vector4d::Zero(),
    bool has_aim_point = false);
#endif

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;

  // 🆕 CBoard指针（用于自动获取robot_id并更新敌方颜色）
  io::CBoard* cboard_ = nullptr;

#ifdef AMENT_CMAKE_FOUND
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<void> marker_pub_;  // 使用 void* 类型擦除，避免暴露 ROS2 具体类型
#endif

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP