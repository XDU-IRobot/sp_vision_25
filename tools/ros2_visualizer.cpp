#ifdef AMENT_CMAKE_FOUND

#include "ros2_visualizer.hpp"

#include <geometry_msgs/msg/point.hpp>

#include "logger.hpp"

namespace tools
{

ROS2Visualizer::ROS2Visualizer(const std::string & node_name, const std::string & marker_topic)
: running_(true)
{
  // 创建ROS2节点
  node_ = std::make_shared<rclcpp::Node>(node_name);

  // 初始化TF2广播器
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // 初始化Marker发布器
  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
  marker_array_publisher_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic + "_array", 10);

  // 启动独立spin线程
  spin_thread_ = std::make_unique<std::thread>([this]() {
    logger()->info("[ROS2Visualizer] Spin thread started");
    while (running_ && rclcpp::ok()) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1ms轮询，低延迟
    }
    logger()->info("[ROS2Visualizer] Spin thread stopped");
  });

  logger()->info("[ROS2Visualizer] Initialized: node={}, marker_topic={}", node_name, marker_topic);
}

ROS2Visualizer::~ROS2Visualizer()
{
  running_ = false;
  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }
  logger()->info("[ROS2Visualizer] Shutdown complete");
}

// ==================== TF2发布实现 ====================

void ROS2Visualizer::publish_static_tf(
  const std::string & parent_frame,
  const std::string & child_frame,
  const Eigen::Matrix3d & R,
  const Eigen::Vector3d & t)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = node_->get_clock()->now();
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;

  transform.transform.translation.x = t.x();
  transform.transform.translation.y = t.y();
  transform.transform.translation.z = t.z();

  transform.transform.rotation = eigen_to_quaternion_msg(R);

  static_tf_broadcaster_->sendTransform(transform);
  logger()->info("[ROS2Visualizer] Published static TF: {} -> {}", parent_frame, child_frame);
}

void ROS2Visualizer::publish_dynamic_tf(
  const std::string & parent_frame,
  const std::string & child_frame,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & t,
  const rclcpp::Time & timestamp)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = timestamp;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;

  transform.transform.translation.x = t.x();
  transform.transform.translation.y = t.y();
  transform.transform.translation.z = t.z();

  transform.transform.rotation = eigen_to_quaternion_msg(q);

  dynamic_tf_broadcaster_->sendTransform(transform);
}

void ROS2Visualizer::publish_dynamic_tf(
  const std::string & parent_frame,
  const std::string & child_frame,
  const Eigen::Matrix3d & R,
  const Eigen::Vector3d & t,
  const rclcpp::Time & timestamp)
{
  Eigen::Quaterniond q(R);
  publish_dynamic_tf(parent_frame, child_frame, q, t, timestamp);
}

// ==================== Marker发布实现 ====================

void ROS2Visualizer::publish_marker(const visualization_msgs::msg::Marker & marker)
{
  marker_publisher_->publish(marker);
}

void ROS2Visualizer::publish_marker_array(const visualization_msgs::msg::MarkerArray & marker_array)
{
  marker_array_publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker ROS2Visualizer::create_sphere_marker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  double x, double y, double z,
  double r, double g, double b, double a,
  double scale,
  const rclcpp::Time & timestamp)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  return marker;
}

visualization_msgs::msg::Marker ROS2Visualizer::create_arrow_marker(
  const std::string & frame_id,
  const std::string & ns,
  int id,
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & end,
  double r, double g, double b, double a,
  double shaft_diameter,
  double head_diameter,
  const rclcpp::Time & timestamp)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 箭头使用两个点：起点和终点
  geometry_msgs::msg::Point p1, p2;
  p1.x = start.x();
  p1.y = start.y();
  p1.z = start.z();
  p2.x = end.x();
  p2.y = end.y();
  p2.z = end.z();
  marker.points.push_back(p1);
  marker.points.push_back(p2);

  marker.scale.x = shaft_diameter;   // 箭杆直径
  marker.scale.y = head_diameter;    // 箭头直径
  marker.scale.z = 0.0;              // 未使用

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  return marker;
}

// ==================== 辅助函数 ====================

geometry_msgs::msg::Quaternion ROS2Visualizer::eigen_to_quaternion_msg(const Eigen::Matrix3d & R)
{
  Eigen::Quaterniond q(R);
  return eigen_to_quaternion_msg(q);
}

geometry_msgs::msg::Quaternion ROS2Visualizer::eigen_to_quaternion_msg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

}  // namespace tools

#endif  // AMENT_CMAKE_FOUND
