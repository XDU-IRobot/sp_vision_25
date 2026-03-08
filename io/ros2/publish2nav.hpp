#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const Eigen::Vector4d & data);

  // TF2发布接口
  void publish_static_gimbal_to_camera(
    const Eigen::Matrix3d & R_camera2gimbal,
    const Eigen::Vector3d & t_camera2gimbal);

  void publish_world_to_gimbal(
    const Eigen::Matrix3d & R_gimbal2world,
    const rclcpp::Time & timestamp);

  void publish_world_to_gimbal(
    const Eigen::Quaterniond & q_gimbal,
    const rclcpp::Time & timestamp);

private:
  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // TF2 广播器
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

  // 辅助函数
  geometry_msgs::msg::Quaternion eigen_to_quaternion_msg(const Eigen::Matrix3d & R);
};

}  // namespace io

#endif  // Publish2Nav_HPP_
