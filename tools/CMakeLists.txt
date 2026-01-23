cmake_minimum_required(VERSION 3.16)

set(TOOLS_SOURCES
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
)

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_geometry_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(visualization_msgs REQUIRED)

# target_include_directories(tools PUBLIC
#     ${rclcpp_INCLUDE_DIRS}
#     ${tf2_ros_INCLUDE_DIRS}
#     ${tf2_geometry_msgs_INCLUDE_DIRS}
#     ${geometry_msgs_INCLUDE_DIRS}
#     ${visualization_msgs_INCLUDE_DIRS}
# )
# target_link_libraries(tools 
#     PUBLIC 
#     rclcpp::rclcpp
#     ${tf2_ros_TARGETS}
#     ${tf2_geometry_msgs_TARGETS}
#     ${geometry_msgs_TARGETS}
#     ${visualization_msgs_TARGETS}
# )
if(ament_cmake_FOUND AND rclcpp_FOUND AND tf2_ros_FOUND AND visualization_msgs_FOUND AND geometry_msgs_FOUND)
    set(ROS2_AVAILABLE TRUE)
    message(STATUS " tools: ROS2 found, enabling tf_publisher and marker_publisher")
    list(APPEND TOOLS_SOURCES
        tf_publisher.cpp
        marker_publisher.cpp
    )
else()
    set(ROS2_AVAILABLE FALSE)
    message(STATUS " tools: ROS2 not found, using stub implementations")
endif()
add_library(tools OBJECT ${TOOLS_SOURCES})
target_include_directories(tools PUBLIC
    ${CMAKE_SOURCE_DIR}
)
if(ROS2_AVAILABLE)
    target_compile_definitions(tools PUBLIC AMENT_CMAKE_FOUND)
    ament_target_dependencies(tools PUBLIC
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        visualization_msgs
)
endif()
