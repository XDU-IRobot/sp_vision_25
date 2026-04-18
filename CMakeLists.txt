cmake_minimum_required(VERSION 3.16.3)

# 设置 CUDA 架构策略（必须在 project() 之前）
if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()

# 检测 CUDA 是否可用（使用现代 CMake 方法）
# 先尝试设置 CUDA 编译器路径（Jetson 和常见安装路径）
if(NOT DEFINED CMAKE_CUDA_COMPILER)
  find_program(CMAKE_CUDA_COMPILER
    NAMES nvcc
    PATHS
      /usr/local/cuda-12.6/bin
      /usr/local/cuda-12.4/bin
      /usr/local/cuda-12/bin
      /usr/local/cuda-11.8/bin
      /usr/local/cuda/bin
    NO_DEFAULT_PATH
  )
  if(CMAKE_CUDA_COMPILER)
    message(STATUS "Found CUDA compiler: ${CMAKE_CUDA_COMPILER}")
  endif()
endif()

include(CheckLanguage)
check_language(CUDA)

if(CMAKE_CUDA_COMPILER)
  # 提前设置 CUDA 架构（必须在 project() 之前）
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    # 检测系统处理器类型来设置合适的 CUDA 架构
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
      # Jetson 设备：Orin (87) 或 Xavier (72)
      set(CMAKE_CUDA_ARCHITECTURES "72;87")
    else()
      # x86_64 设备：常见桌面 GPU
      set(CMAKE_CUDA_ARCHITECTURES "75;86;89")
    endif()
  endif()

  # 在 project() 中启用 CUDA 语言
  project(sp_vision LANGUAGES CXX CUDA)
  message(STATUS "CUDA enabled with architectures: ${CMAKE_CUDA_ARCHITECTURES}")
  set(CUDA_FOUND TRUE)
else()
  # 如果没有 CUDA，只启用 C++ 语言
  project(sp_vision LANGUAGES CXX)
  message(STATUS "CUDA not found, building without CUDA support")
  set(CUDA_FOUND FALSE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
add_definitions(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")


find_package(OpenCV 4.5 REQUIRED)

find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
# 尝试查找 OpenVINO（可选）
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO QUIET COMPONENTS Runtime)

if(OpenVINO_FOUND)
  message(STATUS "OpenVINO found: Inference backend OpenVINO enabled")
else()
  message(STATUS "OpenVINO not found: OpenVINO backend disabled")
endif()

# 检测系统架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(ARCH_SUFFIX "x86_64-linux-gnu")
  set(CUDA_ARCH_DIR "x86_64-linux")
  message(STATUS "Detected architecture: x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(ARCH_SUFFIX "aarch64-linux-gnu")
  set(CUDA_ARCH_DIR "aarch64-linux")
  message(STATUS "Detected architecture: ARM64 (aarch64)")
else()
  message(WARNING "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}, defaulting to x86_64")
  set(ARCH_SUFFIX "x86_64-linux-gnu")
  set(CUDA_ARCH_DIR "x86_64-linux")
endif()

# 尝试查找 TensorRT（可选）
if(CUDA_FOUND)
  # 从 CMAKE_CUDA_COMPILER 推导 CUDA toolkit 目录
  get_filename_component(CUDA_COMPILER_BIN "${CMAKE_CUDA_COMPILER}" DIRECTORY)
  get_filename_component(CUDA_TOOLKIT_ROOT_DIR "${CUDA_COMPILER_BIN}" DIRECTORY)

  message(STATUS "CUDA compiler: ${CMAKE_CUDA_COMPILER}")
  message(STATUS "CUDA toolkit root: ${CUDA_TOOLKIT_ROOT_DIR}")
  message(STATUS "Using CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")

  set(CUDA_NVCC_EXECUTABLE ${CMAKE_CUDA_COMPILER})
endif()

if(CUDA_FOUND)
  find_path(TensorRT_INCLUDE_DIR NvInfer.h
    HINTS /usr/include/${ARCH_SUFFIX} /usr/local/cuda/include /usr/local/TensorRT-10.0/include /usr/include
    PATH_SUFFIXES tensorrt) # maybe 10.0.1.6

  # 查找所有需要的 TensorRT 库
  find_library(TensorRT_LIBRARY nvinfer
    HINTS /usr/lib/${ARCH_SUFFIX} /usr/local/cuda/lib64 /usr/local/TensorRT-10.0/lib /usr/lib)
  find_library(TensorRT_PLUGIN_LIBRARY nvinfer_plugin
    HINTS /usr/lib/${ARCH_SUFFIX} /usr/local/cuda/lib64 /usr/local/TensorRT-10.0/lib /usr/lib)
  find_library(TensorRT_ONNX_LIBRARY nvonnxparser
    HINTS /usr/lib/${ARCH_SUFFIX} /usr/local/cuda/lib64 /usr/local/TensorRT-10.0/lib /usr/lib)

  # 查找 CUDA 运行时库
  find_library(CUDA_RUNTIME_LIBRARY cudart
    HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64
          /usr/local/cuda/lib64
          /usr/local/cuda-12.6/targets/${CUDA_ARCH_DIR}/lib
          /usr/local/cuda-12.4/targets/${CUDA_ARCH_DIR}/lib
          /usr/local/cuda-11.8/targets/${CUDA_ARCH_DIR}/lib
          /usr/local/cuda/targets/${CUDA_ARCH_DIR}/lib)

  if(TensorRT_INCLUDE_DIR AND TensorRT_LIBRARY AND TensorRT_PLUGIN_LIBRARY AND TensorRT_ONNX_LIBRARY AND CUDA_RUNTIME_LIBRARY)
    set(TensorRT_FOUND TRUE)
    set(TensorRT_LIBRARIES ${TensorRT_LIBRARY} ${TensorRT_PLUGIN_LIBRARY} ${TensorRT_ONNX_LIBRARY} ${CUDA_RUNTIME_LIBRARY})
    message(STATUS "TensorRT found: Inference backend TensorRT enabled")
    message(STATUS "  Include: ${TensorRT_INCLUDE_DIR}")
    message(STATUS "  nvinfer: ${TensorRT_LIBRARY}")
    message(STATUS "  nvinfer_plugin: ${TensorRT_PLUGIN_LIBRARY}")
    message(STATUS "  nvonnxparser: ${TensorRT_ONNX_LIBRARY}")
    message(STATUS "  cudart: ${CUDA_RUNTIME_LIBRARY}")
  else()
    set(TensorRT_FOUND FALSE)
    message(STATUS "TensorRT not found: TensorRT backend disabled")
    if(NOT TensorRT_INCLUDE_DIR)
      message(STATUS "  Missing: TensorRT include directory")
    endif()
    if(NOT TensorRT_LIBRARY)
      message(STATUS "  Missing: nvinfer library")
    endif()
    if(NOT TensorRT_PLUGIN_LIBRARY)
      message(STATUS "  Missing: nvinfer_plugin library")
    endif()
    if(NOT TensorRT_ONNX_LIBRARY)
      message(STATUS "  Missing: nvonnxparser library")
    endif()
    if(NOT CUDA_RUNTIME_LIBRARY)
      message(STATUS "  Missing: CUDA runtime library")
    endif()
  endif()
else()
  message(STATUS "CUDA not found: TensorRT backend disabled")
endif()

# 检查至少有一个后端可用
if(NOT OpenVINO_FOUND AND NOT TensorRT_FOUND)
  message(WARNING "Neither OpenVINO nor TensorRT found. YOLO inference will not be available.")
endif()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# 提前检查 ROS2 环境（tools 需要这些信息来决定是否编译 ROS2 Visualizer）
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(visualization_msgs QUIET)
find_package(tf2_ros QUIET)
find_package(geometry_msgs QUIET)

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)
add_subdirectory(librm)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(daheng_camera_test src/daheng_camera_test.cpp)
add_executable(uav_debug src/uav_debug.cpp)
add_executable(uav_debug_jetsn src/uav_debug_jetsn.cpp)
add_executable(uav_debug_mpc src/uav_debug_mpc.cpp)

# 需要 auto_buff 的可执行文件（需要 OpenVINO 和 Ceres）
if(OpenVINO_FOUND AND Ceres_FOUND)
    add_executable(uav src/uav.cpp)
    add_executable(mt_standard src/mt_standard.cpp)
    add_executable(standard_mpc src/standard_mpc.cpp)
    add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
    add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
endif()

# auto_buff 相关可执行文件（需要 OpenVINO 和 Ceres）
if(OpenVINO_FOUND AND Ceres_FOUND)
    add_executable(auto_buff_debug src/auto_buff_debug.cpp)
    add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
    target_link_libraries(auto_buff_debug fmt::fmt yaml-cpp auto_buff tools io ${OpenCV_LIBS} ${CERES_LIBRARIES})
    target_link_libraries(auto_buff_debug_mpc fmt::fmt yaml-cpp auto_buff tools io ${OpenCV_LIBS} ${CERES_LIBRARIES})
    target_link_libraries(uav fmt::fmt yaml-cpp auto_aim auto_buff tools io ${OpenCV_LIBS})
    target_link_libraries(mt_standard fmt::fmt yaml-cpp auto_aim auto_buff tools io ${OpenCV_LIBS})
    target_link_libraries(standard_mpc fmt::fmt yaml-cpp auto_aim auto_buff tools io ${OpenCV_LIBS})
    target_link_libraries(auto_aim_debug_mpc fmt::fmt yaml-cpp auto_aim auto_buff tools io ${OpenCV_LIBS})
    target_link_libraries(mt_auto_aim_debug fmt::fmt yaml-cpp auto_aim auto_buff tools io ${OpenCV_LIBS})
endif()

target_link_libraries(standard fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})
target_link_libraries(daheng_camera_test fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})
target_link_libraries(uav_debug fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})
if(ament_cmake_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND)
  target_link_libraries(uav_debug_jetsn fmt::fmt yaml-cpp auto_aim tools io rclcpp::rclcpp ${OpenCV_LIBS})
  ament_target_dependencies(uav_debug_jetsn rclcpp sensor_msgs)
  target_link_libraries(uav_debug_mpc fmt::fmt yaml-cpp auto_aim tools io rclcpp::rclcpp ${OpenCV_LIBS})
  ament_target_dependencies(uav_debug_mpc rclcpp sensor_msgs)
else()
  target_link_libraries(uav_debug_jetsn fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})
  target_link_libraries(uav_debug_mpc fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})
endif()

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture fmt::fmt tools io ${OpenCV_LIBS})
target_link_libraries(calibrate_camera fmt::fmt yaml-cpp tools ${OpenCV_LIBS})
target_link_libraries(calibrate_handeye fmt::fmt yaml-cpp tools ${OpenCV_LIBS})
target_link_libraries(calibrate_robotworld_handeye fmt::fmt yaml-cpp tools ${OpenCV_LIBS})
target_link_libraries(split_video fmt::fmt tools ${OpenCV_LIBS})

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_pnp_test tests/camera_pnp_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)

target_link_libraries(auto_aim_test fmt::fmt yaml-cpp tools io auto_aim ${OpenCV_LIBS})
target_link_libraries(camera_detect_test fmt::fmt yaml-cpp tools auto_aim io ${OpenCV_LIBS})

# auto_buff 测试（需要 OpenVINO 和 Ceres）
if(OpenVINO_FOUND AND Ceres_FOUND)
    add_executable(auto_buff_test tests/auto_buff_test.cpp)
    target_link_libraries(auto_buff_test fmt::fmt yaml-cpp auto_buff tools io ${OpenCV_LIBS} ${CERES_LIBRARIES})
endif()

# minimum_vision_system 测试（需要 OpenVINO，使用 MultiThreadDetector）
if(OpenVINO_FOUND)
    add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
    target_link_libraries(minimum_vision_system fmt::fmt yaml-cpp tools io auto_aim ${OpenCV_LIBS})
endif()
target_link_libraries(camera_pnp_test fmt::fmt yaml-cpp tools auto_aim io ${OpenCV_LIBS})
target_link_libraries(camera_test fmt::fmt tools io ${OpenCV_LIBS})
target_link_libraries(camera_thread_test fmt::fmt auto_aim tools io ${OpenCV_LIBS})
target_link_libraries(cboard_test fmt::fmt tools io ${OpenCV_LIBS})
target_link_libraries(fire_test fmt::fmt tools io ${OpenCV_LIBS})
target_link_libraries(detector_video_test fmt::fmt yaml-cpp tools auto_aim ${OpenCV_LIBS})
target_link_libraries(gimbal_response_test fmt::fmt yaml-cpp tools io ${OpenCV_LIBS})
target_link_libraries(multi_usbcamera_test fmt::fmt tools io ${OpenCV_LIBS})
target_link_libraries(usbcamera_detect_test fmt::fmt yaml-cpp tools io auto_aim ${OpenCV_LIBS})
target_link_libraries(usbcamera_test fmt::fmt yaml-cpp tools io ${OpenCV_LIBS})
target_link_libraries(handeye_test fmt::fmt yaml-cpp tools io auto_aim ${OpenCV_LIBS})
target_link_libraries(dm_test fmt::fmt yaml-cpp tools io ${OpenCV_LIBS})

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})

add_executable(planner_test_debug tests/planner_test_debug.cpp)
target_link_libraries(planner_test_debug fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})

add_executable(camera_planner_simple tests/camera_planner_simple.cpp)
target_link_libraries(camera_planner_simple fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})

add_executable(camera_planner_realtime tests/camera_planner_realtime.cpp)
target_link_libraries(camera_planner_realtime fmt::fmt yaml-cpp auto_aim tools io ${OpenCV_LIBS})

add_executable(trigger_test tests/trigger_test.cpp)
target_link_libraries(trigger_test fmt::fmt yaml-cpp tools io ${OpenCV_LIBS})

# ROS2 环境已在前面检查（为了让 tools 库能正确编译 ROS2 Visualizer）
# 只编译 standard_ros2 (不编译 sentry 等其他 ROS2 程序)
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND visualization_msgs_FOUND AND tf2_ros_FOUND AND geometry_msgs_FOUND)
    message(STATUS "ROS2 environment found, compiling standard_ros2 with marker visualization")

    add_executable(standard_ros2 src/standard.cpp)
    target_link_libraries(standard_ros2 fmt::fmt yaml-cpp auto_aim auto_buff tools io rclcpp::rclcpp ${OpenCV_LIBS})
    ament_target_dependencies(standard_ros2 rclcpp std_msgs visualization_msgs tf2_ros geometry_msgs)
    target_compile_definitions(standard_ros2 PRIVATE AMENT_CMAKE_FOUND)

    # 同时编译 auto_aim_test_ros2
    add_executable(auto_aim_test_ros2 tests/auto_aim_test.cpp)
    target_link_libraries(auto_aim_test_ros2 fmt::fmt yaml-cpp tools io auto_aim rclcpp::rclcpp ${OpenCV_LIBS})
    ament_target_dependencies(auto_aim_test_ros2 rclcpp std_msgs visualization_msgs tf2_ros geometry_msgs)
    target_compile_definitions(auto_aim_test_ros2 PRIVATE AMENT_CMAKE_FOUND)
else()
    message(STATUS "ROS2 environment not complete, skipping standard_ros2")
endif()

# 原有的 sentry 等程序编译（已注释）
# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
# if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND visualization_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
#     # 编译 sentry 和 publish_test
#     add_executable(sentry src/sentry.cpp)
#     add_executable(sentry_bp src/sentry_bp.cpp)
#     add_executable(sentry_debug src/sentry_debug.cpp)
#     add_executable(sentry_multithread src/sentry_multithread.cpp)
#     add_executable(publish_test tests/publish_test.cpp)
#     add_executable(subscribe_test tests/subscribe_test.cpp)
#     add_executable(topic_loop_test tests/topic_loop_test.cpp)
#
#     target_link_libraries(sentry fmt::fmt yaml-cpp auto_aim omniperception tools io ${OpenCV_LIBS})
#     target_link_libraries(sentry_bp fmt::fmt yaml-cpp auto_aim omniperception tools io ${OpenCV_LIBS})
#     target_link_libraries(sentry_debug fmt::fmt yaml-cpp auto_aim omniperception tools io ${OpenCV_LIBS})
#     target_link_libraries(sentry_multithread fmt::fmt yaml-cpp auto_aim omniperception tools io ${OpenCV_LIBS})
#     target_link_libraries(publish_test fmt::fmt tools io auto_aim ${OpenCV_LIBS})
#     target_link_libraries(subscribe_test fmt::fmt tools io ${OpenCV_LIBS})
#     target_link_libraries(topic_loop_test fmt::fmt tools io ${OpenCV_LIBS})
# else()
#     message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
# endif()
