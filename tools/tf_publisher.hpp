#ifndef TOOLS__TF_PUBLISHER_HPP
#define TOOLS__TF_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <memory>

namespace tools
{
/**
 * @brief TF 发布器类
 * 
 * 用于发布坐标变换信息，构建 TF 树。
 */
class TFPublisher
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2 节点指针
     */
    explicit TFPublisher(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief 发布变换
     * @param parent_frame 父坐标系名称
     * @param child_frame 子坐标系名称
     * @param translation 平移向量
     * @param rotation 旋转四元数
     */
    void publishTransform(
        const std::string & parent_frame,
        const std::string & child_frame,
        const Eigen::Vector3d & translation,
        const Eigen::Quaterniond & rotation
    );

    /**
     * @brief 发布变换（使用旋转矩阵）
     * @param parent_frame 父坐标系名称
     * @param child_frame 子坐标系名称
     * @param translation 平移向量
     * @param rotation_matrix 旋转矩阵
     */
    void publishTransform(
        const std::string & parent_frame,
        const std::string & child_frame,
        const Eigen::Vector3d & translation,
        const Eigen::Matrix3d & rotation_matrix
    );

    /**
     * @brief 发布变换（使用 OpenCV 格式）
     * @param parent_frame 父坐标系名称
     * @param child_frame 子坐标系名称
     * @param rvec 旋转向量（Rodrigues）
     * @param tvec 平移向量
     */
    void publishTransform(
        const std::string & parent_frame,
        const std::string & child_frame,
        const cv::Vec3d & rvec,
        const cv::Vec3d & tvec
    );

private:
    rclcpp::Node::SharedPtr node_;  // 保存节点指针用于获取时间戳
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    geometry_msgs::msg::Quaternion eigenToMsgQuaternion(const Eigen::Quaterniond & q);
};

}  // namespace tools

#endif  // TOOLS__TF_PUBLISHER_HPP
