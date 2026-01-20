#ifndef MARKER_PUBLISHER_HPP_
#define MARKER_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>

namespace tools
{

class MarkerPublisher
{
public:
  explicit MarkerPublisher(rclcpp::Node::SharedPtr node);
  void publishArmorMarker(const std::string &frame_id, int id,
                             const Eigen::Vector3d &position,
                             const Eigen::Quaterniond &orientation,
                             double width, double height,
                             int r = 0 ,int g = 0, int b = 255, double alpha = 0.8);
  void publishSphereMarker(const std::string &frame_id, int id,
                           const Eigen::Vector3d &position, double radius,
                           int r = 255, int g = 255, int b = 0, double alpha = 0.8);
  void publishCircleMarker(const std::string &frame_id, int id,
                           const Eigen::Vector3d &center_position, double radius,
                           int r = 255, int g = 255, int b = 0, double alpha = 0.3);
  void publishArrowMarker(const std::string &frame_id, int id,
                           const Eigen::Vector3d &start,
                           const Eigen::Vector3d &end,
                           int r = 0, int g = 255, int b = 255, double alpha = 1.0);
  void clearMarkers();
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace tools

#endif  // MARKER_PUBLISHER_HPP_