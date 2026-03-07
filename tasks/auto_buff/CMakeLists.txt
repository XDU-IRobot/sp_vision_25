cmake_minimum_required(VERSION 3.16)

# auto_buff 模块需要 OpenVINO 和 Ceres 支持
find_package(Ceres QUIET)

if(OpenVINO_FOUND AND Ceres_FOUND)
    message(STATUS "[auto_buff] Building with OpenVINO and Ceres support")

    add_library(auto_buff OBJECT
        buff_type.cpp
        buff_detector.cpp
        buff_solver.cpp
        buff_aimer.cpp
        buff_target.cpp
        yolo11_buff.cpp
    )

    target_compile_definitions(auto_buff PRIVATE ENABLE_OPENVINO)
    target_link_libraries(auto_buff auto_aim openvino::runtime ${CERES_LIBRARIES})
else()
    if(NOT OpenVINO_FOUND)
        message(STATUS "[auto_buff] OpenVINO not found, skipping auto_buff module")
    endif()
    if(NOT Ceres_FOUND)
        message(STATUS "[auto_buff] Ceres not found, skipping auto_buff module")
    endif()

    # 创建一个空的目标以避免链接错误
    add_library(auto_buff INTERFACE)
endif()