#ifndef TOOLS__ROS2_VISUALIZER_HPP
#define TOOLS__ROS2_VISUALIZER_HPP

#ifdef AMENT_CMAKE_FOUND

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tools
{

/**
 * @brief ROS2可视化工具类 - 提供TF2和Marker发布功能
 *
 * 特性：
 * - 异步发布，不阻塞主循环
 * - 独立线程处理ROS2 spin
 * - 支持静态和动态TF2变换
 * - 支持Marker和MarkerArray发布
 * - 线程安全
 */
class ROS2Visualizer
{
public:
  /**
   * @brief 构造函数
   * @param node_name ROS2节点名称
   * @param marker_topic Marker发布话题名称
   */
  explicit ROS2Visualizer(
    const std::string & node_name = "visualization_node",
    const std::string & marker_topic = "visualization_markers");

  ~ROS2Visualizer();

  // ==================== TF2发布接口 ====================

  /**
   * @brief 发布静态TF变换（如相机标定参数）
   * @param parent_frame 父坐标系
   * @param child_frame 子坐标系
   * @param R 旋转矩阵（child相对于parent）
   * @param t 平移向量（parent到child）
   */
  void publish_static_tf(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Eigen::Matrix3d & R,
    const Eigen::Vector3d & t);

  /**
   * @brief 发布动态TF变换（如云台姿态）
   * @param parent_frame 父坐标系
   * @param child_frame 子坐标系
   * @param q 四元数（child相对于parent）
   * @param t 平移向量（parent到child）
   * @param timestamp 时间戳
   */
  void publish_dynamic_tf(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & t,
    const rclcpp::Time & timestamp);

  /**
   * @brief 发布动态TF变换（旋转矩阵版本）
   */
  void publish_dynamic_tf(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Eigen::Matrix3d & R,
    const Eigen::Vector3d & t,
    const rclcpp::Time & timestamp);

  // ==================== Marker发布接口 ====================

  /**
   * @brief 发布单个Marker
   */
  void publish_marker(const visualization_msgs::msg::Marker & marker);

  /**
   * @brief 发布MarkerArray
   */
  void publish_marker_array(const visualization_msgs::msg::MarkerArray & marker_array);

  /**
   * @brief 创建球体Marker的辅助函数
   * @param frame_id 坐标系
   * @param ns 命名空间
   * @param id Marker ID
   * @param x, y, z 位置
   * @param r, g, b, a 颜色和透明度
   * @param scale 尺寸
   * @param timestamp 时间戳
   */
  visualization_msgs::msg::Marker create_sphere_marker(
    const std::string & frame_id,
    const std::string & ns,
    int id,
    double x, double y, double z,
    double r, double g, double b, double a,
    double scale,
    const rclcpp::Time & timestamp);

  /**
   * @brief 创建箭头Marker的辅助函数
   */
  visualization_msgs::msg::Marker create_arrow_marker(
    const std::string & frame_id,
    const std::string & ns,
    int id,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & end,
    double r, double g, double b, double a,
    double shaft_diameter,
    double head_diameter,
    const rclcpp::Time & timestamp);

  /**
   * @brief 获取ROS2节点指针（用于高级自定义）
   */
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }

private:
  // ROS2核心
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<std::thread> spin_thread_;

  // TF2广播器
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

  // Marker发布器
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

  // 线程控制
  std::atomic<bool> running_;

  // 辅助函数
  geometry_msgs::msg::Quaternion eigen_to_quaternion_msg(const Eigen::Matrix3d & R);
  geometry_msgs::msg::Quaternion eigen_to_quaternion_msg(const Eigen::Quaterniond & q);
};

}  // namespace tools

#endif  // AMENT_CMAKE_FOUND

#endif  // TOOLS__ROS2_VISUALIZER_HPP
