#include "marker_publisher.hpp"

namespace tools
{
MarkerPublisher::MarkerPublisher(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "armor_markers", 10);
}
void MarkerPublisher::publishArmorMarker(
    const std::string & frame_id,
    int id,
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation,
    double width = 0.1,
    double height = 0.1,
    int r,
    int g,
    int b,
    double alpha)
{
   visualization_msgs::msg::MarkerArray marker_array;  
   visualization_msgs::msg::Marker marker;
   marker.header.frame_id = frame_id;
   marker.header.stamp = node_->now();
   marker.ns = "armor";
   marker.id = id;
   marker.type = visualization_msgs::msg::Marker::CUBE;
   marker.action = visualization_msgs::msg::Marker::ADD;
   marker.pose.position.x = position.x();
   marker.pose.position.y = position.y();
   marker.pose.position.z = position.z();
   marker.pose.orientation.x = orientation.x();
   marker.pose.orientation.y = orientation.y();
   marker.pose.orientation.z = orientation.z();
   marker.pose.orientation.w = orientation.w();
   // 装甲板坐标系：x=厚度（薄），y=宽度，z=高度
   // 使用相同的尺寸使其呈现正方形
   double size = std::max(width, height);
   marker.scale.x = 0.01;   // x轴：厚度方向（非常薄）
   marker.scale.y = size;   // y轴：宽度方向（正方形）
   marker.scale.z = size;   // z轴：高度方向（正方形）
   marker.color.r = 1.0;   // 纯红色
   marker.color.g = 0.0;
   marker.color.b = 0.0;
   marker.color.a = alpha;
   marker.lifetime = rclcpp::Duration::from_seconds(0.2);
   marker_array.markers.push_back(marker);

   // 边框
   visualization_msgs::msg::Marker edge_marker;
   edge_marker.header.frame_id = frame_id;
    edge_marker.header.stamp = node_->now();
    edge_marker.ns = "armor_edge";
    edge_marker.id = id ;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.pose  = marker.pose;
    edge_marker.scale.x = 0.003; // 线条宽度
    edge_marker.color.r = 1.0;
    edge_marker.color.g = 1.0;
    edge_marker.color.b = 0.0;
    edge_marker.color.a = 1.0;
    edge_marker.lifetime = marker.lifetime;
    // 使用之前计算的正方形尺寸
    double half_size = size / 2.0;
    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
  // 装甲板在 yz 平面，x=0（厚度方向）
  p1.x = 0; p1.y = -half_size; p1.z = -half_size;
  p2.x = 0; p2.y =  half_size; p2.z = -half_size;
  p3.x = 0; p3.y =  half_size; p3.z =  half_size;
  p4.x = 0; p4.y = -half_size; p4.z =  half_size;
  p5 = p1;

  edge_marker.points = {p1, p2, p3, p4, p5};

  marker_array.markers.push_back(edge_marker);
  marker_pub_->publish(marker_array);
}
void MarkerPublisher::publishSphereMarker(
    const std::string & frame_id,
    int id,
    const Eigen::Vector3d & position,
    double radius,
    int r,
    int g,
    int b,
    double alpha)
{
   visualization_msgs::msg::MarkerArray marker_array;  
   visualization_msgs::msg::Marker marker;
   marker.header.frame_id = frame_id;
   marker.header.stamp = node_->now();
   marker.ns = "sphere";
   marker.id = id;
   marker.type = visualization_msgs::msg::Marker::SPHERE;
   marker.action = visualization_msgs::msg::Marker::ADD;
   marker.pose.position.x = position.x();
   marker.pose.position.y = position.y();
   marker.pose.position.z = position.z();
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0;
   marker.scale.x = radius * 0.3;  // 球体大小：从 2*radius 改为 0.3*radius（更小）
   marker.scale.y = radius * 0.3;
   marker.scale.z = radius * 0.3;
   marker.color.r = 0.0;   // 绿色
   marker.color.g = 1.0;
   marker.color.b = 0.0;
   marker.color.a = alpha;
   marker.lifetime = rclcpp::Duration::from_seconds(0.2);
   marker_array.markers.push_back(marker);

   marker_pub_->publish(marker_array);
}
void MarkerPublisher::publishCircleMarker(
    const std::string & frame_id,
    int id,
    const Eigen::Vector3d & position,
    double radius,
    int r,
    int g,
    int b,
    double alpha)
{
   visualization_msgs::msg::MarkerArray marker_array;  
   visualization_msgs::msg::Marker marker;
   marker.header.frame_id = frame_id;
   marker.header.stamp = node_->now();
   marker.ns = "circle";
   marker.id = id;
   marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
   marker.action = visualization_msgs::msg::Marker::ADD;
   marker.pose.position.x = position.x();
   marker.pose.position.y = position.y();
   marker.pose.position.z = position.z();
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
   marker.pose.orientation.z = 0.0;
   marker.pose.orientation.w = 1.0;
   marker.scale.x = 0.01; // 线宽
   marker.color.r = r;
   marker.color.g = g;
   marker.color.b = b;
   marker.color.a = alpha;

   // 生成圆的点
   int num_points = 36;
   for (int i = 0; i <= num_points; ++i) {
     double angle = i * 2.0 * M_PI / num_points;
     geometry_msgs::msg::Point p;
     p.x = radius * cos(angle);
     p.y = radius * sin(angle);
     p.z = 0.0;
     marker.points.push_back(p);
   }

   marker.lifetime = rclcpp::Duration::from_seconds(0.2);
   marker_array.markers.push_back(marker);

   marker_pub_->publish(marker_array);
}
void MarkerPublisher::publishArrowMarker(
    const std::string & frame_id,
    int id,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & end,
    int r, int g, int b, double alpha)
{
   visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "arrow";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start_point;
  start_point.x = start.x();
  start_point.y = start.y();
  start_point.z = start.z();

  geometry_msgs::msg::Point end_point;
  end_point.x = end.x();
  end_point.y = end.y();
  end_point.z = end.z();

  marker.points.push_back(start_point);
  marker.points.push_back(end_point);

  marker.scale.x = 0.02;
  marker.scale.y = 0.04;
  marker.scale.z = 0.1;

  marker.color.r = r / 255.0;
  marker.color.g = g / 255.0;
  marker.color.b = b / 255.0;
  marker.color.a = alpha;

  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);
}
void MarkerPublisher::clearMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);
}
}  // namespace tools