cmake_minimum_required(VERSION 3.16)

# 基础源文件列表
set(TOOLS_SOURCES
    exiter.cpp
    extended_kalman_filter.cpp
    errState_extended_kalman.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    plot_record.cpp
    trajectory.cpp
    recorder.cpp
    debug_recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
    float16.cpp
)

# 检查ROS2包是否可用（在父CMakeLists.txt中已经find_package）
if(rclcpp_FOUND AND tf2_ros_FOUND AND geometry_msgs_FOUND AND visualization_msgs_FOUND)
    list(APPEND TOOLS_SOURCES ros2_visualizer.cpp)
    message(STATUS "[Tools] ROS2 Visualizer enabled")
endif()

add_library(tools STATIC ${TOOLS_SOURCES})

# 如果ROS2可用，先设置ROS2依赖（ament_target_dependencies必须在其他链接之前）
if(rclcpp_FOUND AND tf2_ros_FOUND AND geometry_msgs_FOUND AND visualization_msgs_FOUND)
    # 定义AMENT_CMAKE_FOUND宏，让ros2_visualizer.cpp中的代码被编译
    target_compile_definitions(tools PUBLIC AMENT_CMAKE_FOUND)

    ament_target_dependencies(tools PUBLIC
        rclcpp
        tf2_ros
        geometry_msgs
        visualization_msgs
    )
endif()

if(rclcpp_FOUND AND sensor_msgs_FOUND AND std_msgs_FOUND)
    target_compile_definitions(tools PUBLIC TOOLS_HAS_ROS2_DEBUG_RECORDER)

    ament_target_dependencies(tools PUBLIC
        rclcpp
        sensor_msgs
        std_msgs
    )

    message(STATUS "[Tools] ROS2 DebugRecorder enabled")
endif()

# 链接fmt库以解决undefined reference错误
target_link_libraries(tools PUBLIC fmt::fmt nlohmann_json::nlohmann_json)
