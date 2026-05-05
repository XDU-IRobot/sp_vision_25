cmake_minimum_required(VERSION 3.16)

# 基础源文件（始终编译）
set(AUTO_AIM_SOURCES
    armor.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    multithread/commandgener.cpp
    planner/planner.cpp
)

# 根据可用后端添加YOLO实现源文件
set(YOLO_SOURCES "")

if(OpenVINO_FOUND)
    message(STATUS "[auto_aim] Adding OpenVINO YOLO implementations")
    list(APPEND YOLO_SOURCES
        classifier.cpp
        multithread/mt_detector.cpp
        yolos/yolov5.cpp
        yolos/yolov8.cpp
        yolos/yolo11.cpp
        yolos/yolo26.cpp
    )
endif()

if(TensorRT_FOUND)
    message(STATUS "[auto_aim] Adding TensorRT YOLO implementations")

    # CUDA 语言已在主 CMakeLists.txt 中启用，这里只需添加源文件
    if(CUDA_NVCC_EXECUTABLE)
        list(APPEND YOLO_SOURCES
            classifier_trt.cpp
            yolos/tensorrt_logger.cpp
            yolos/yolov8_trt.cpp
            yolos/yolo11_trt.cpp
            yolos/yolo26_trt.cpp
            yolos/cuda_preprocess.cu
        )
    else()
        message(WARNING "[auto_aim] TensorRT found but CUDA compiler (nvcc) not available. TensorRT support will be disabled.")
        set(TensorRT_FOUND FALSE)
    endif()
endif()

# 合并所有源文件
list(APPEND AUTO_AIM_SOURCES ${YOLO_SOURCES})

# 检查 ROS2 环境（用于 tracker 的 marker 功能）
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(visualization_msgs QUIET)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT ${AUTO_AIM_SOURCES})

# 基础链接
target_link_libraries(auto_aim io tinympcstatic)

# 条件链接：OpenVINO
if(OpenVINO_FOUND)
    target_compile_definitions(auto_aim PUBLIC ENABLE_OPENVINO)
    target_link_libraries(auto_aim openvino::runtime)
    message(STATUS "[auto_aim] OpenVINO support enabled")
endif()

# 条件链接：TensorRT
if(TensorRT_FOUND)
    target_compile_definitions(auto_aim PUBLIC ENABLE_TENSORRT)

    # 添加 TensorRT include 目录
    target_include_directories(auto_aim PUBLIC ${TensorRT_INCLUDE_DIR})

    # 添加 CUDA include 目录（从 CMAKE_CUDA_COMPILER 推导）
    if(CMAKE_CUDA_COMPILER)
        get_filename_component(CUDA_BIN_DIR "${CMAKE_CUDA_COMPILER}" DIRECTORY)
        get_filename_component(CUDA_ROOT_DIR "${CUDA_BIN_DIR}" DIRECTORY)
        target_include_directories(auto_aim PUBLIC "${CUDA_ROOT_DIR}/include")
        message(STATUS "[auto_aim] CUDA include directory: ${CUDA_ROOT_DIR}/include")
    endif()

    # 链接 TensorRT 库
    target_link_libraries(auto_aim ${TensorRT_LIBRARIES})

    # 设置 CUDA 架构（继承主 CMakeLists 的设置）
    if(CMAKE_CUDA_ARCHITECTURES)
        set_property(TARGET auto_aim PROPERTY CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES})
    endif()

    message(STATUS "[auto_aim] TensorRT support enabled with CUDA preprocessing")
endif()

# 如果 ROS2 可用，添加 ROS2 支持
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND visualization_msgs_FOUND)
    message(STATUS "[auto_aim] ROS2 support enabled for tracker marker visualization")
    target_compile_definitions(auto_aim PRIVATE AMENT_CMAKE_FOUND)
    target_link_libraries(auto_aim rclcpp::rclcpp)
    ament_target_dependencies(auto_aim rclcpp std_msgs visualization_msgs)
else()
    message(STATUS "[auto_aim] ROS2 not available, tracker marker visualization disabled")
endif()