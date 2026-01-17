cmake_minimum_required(VERSION 3.16)

# 检查ros2环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_geometry_msgs QUIET)
find_package(geometry_msgs QUIET)


add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
    tf_publisher.cpp
)
target_link_libraries(tools 
    PUBLIC 
    rclcpp::rclcpp
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
)
