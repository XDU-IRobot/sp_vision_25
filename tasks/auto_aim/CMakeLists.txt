cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# 检查 ROS2 环境（用于 tracker 的 marker 功能）
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(visualization_msgs QUIET)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim io openvino::runtime tinympcstatic)

# 如果 ROS2 可用，添加 ROS2 支持
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND visualization_msgs_FOUND)
    message(STATUS "[auto_aim] ROS2 support enabled for tracker marker visualization")
    target_compile_definitions(auto_aim PRIVATE AMENT_CMAKE_FOUND)
    target_link_libraries(auto_aim rclcpp::rclcpp)
    ament_target_dependencies(auto_aim rclcpp std_msgs visualization_msgs)
else()
    message(STATUS "[auto_aim] ROS2 not available, tracker marker visualization disabled")
endif()