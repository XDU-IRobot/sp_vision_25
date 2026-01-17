#include "tf_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/calib3d.hpp>

namespace tools
{

TFPublisher::TFPublisher(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "armor_markers", 10);
}

void TFPublisher::publishTransform(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation
)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    
    // 手动设置平移
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    
    // 设置旋转
    transform.transform.rotation = eigenToMsgQuaternion(rotation);

    tf_broadcaster_->sendTransform(transform);
}

void TFPublisher::publishTransform(
    const std::string & parent_frame,
    const std::string & child_frame,
    const Eigen::Vector3d & translation,
    const Eigen::Matrix3d & rotation_matrix
)
{
    // 旋转矩阵转四元数
    Eigen::Quaterniond q(rotation_matrix);
    q.normalize();
    publishTransform(parent_frame, child_frame, translation, q);
}

void TFPublisher::publishTransform(
    const std::string & parent_frame,
    const std::string & child_frame,
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec
)
{
    // OpenCV Rodrigues 旋转向量转旋转矩阵
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);

    // 转换为 Eigen
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 
        rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
        rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
        rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2);
    
    Eigen::Vector3d translation(tvec[0], tvec[1], tvec[2]);
    
    publishTransform(parent_frame, child_frame, translation, rotation_matrix);
}
void TFPublisher::publishArmorMarker(
    const std::string & frame_id,
    int armor_id,
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation,
    double width,
    double height,
    int color_r,
    int color_g,
    int color_b
)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->now();
    marker.ns = "armor";
    marker.id = armor_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 位置
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();

    // 姿态
    marker.pose.orientation = eigenToMsgQuaternion(orientation);

    // 尺寸（装甲板是薄板）
    marker.scale.x = width;
    marker.scale.y = height;
    marker.scale.z = 0.005;  // 5mm 厚度

    // 颜色（半透明）
    marker.color.r = color_r / 255.0;
    marker.color.g = color_g / 255.0;
    marker.color.b = color_b / 255.0;
    marker.color.a = 0.8;  // 透明度

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);  // 200ms 后消失

    marker_array.markers.push_back(marker);

    // ✅ 添加装甲板边框（用线条）
    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header = marker.header;
    edge_marker.ns = "armor_edge";
    edge_marker.id = armor_id;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.pose = marker.pose;
    edge_marker.scale.x = 0.003;  // 线宽 3mm

    // 边框颜色（亮色）
    edge_marker.color.r = 1.0;
    edge_marker.color.g = 1.0;
    edge_marker.color.b = 0.0;
    edge_marker.color.a = 1.0;

    edge_marker.lifetime = marker.lifetime;

    // 定义矩形边框的顶点（局部坐标系）
    double hw = width / 2.0;
    double hh = height / 2.0;
    
    geometry_msgs::msg::Point p1, p2, p3, p4, p5;
    p1.x = -hw; p1.y = -hh; p1.z = 0;
    p2.x =  hw; p2.y = -hh; p2.z = 0;
    p3.x =  hw; p3.y =  hh; p3.z = 0;
    p4.x = -hw; p4.y =  hh; p4.z = 0;
    p5 = p1;  // 闭合

    edge_marker.points = {p1, p2, p3, p4, p5};

    marker_array.markers.push_back(edge_marker);

    marker_pub_->publish(marker_array);
}
void TFPublisher::clearMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    
    marker_pub_->publish(marker_array);
}
geometry_msgs::msg::Quaternion TFPublisher::eigenToMsgQuaternion(const Eigen::Quaterniond & q)
{
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

}  // namespace tools


