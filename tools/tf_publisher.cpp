#include "tf_publisher.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tools
{

TFPublisher::TFPublisher(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
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
    
    // 转换为 Eigen
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 
        rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
        rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
        rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2);
    
    Eigen::Vector3d translation(tvec[0], tvec[1], tvec[2]);
    
    publishTransform(parent_frame, child_frame, translation, rotation_matrix);
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


