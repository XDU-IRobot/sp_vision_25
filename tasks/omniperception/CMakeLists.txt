cmake_minimum_required(VERSION 3.16)

# omniperception 模块需要 OpenVINO 支持
if(OpenVINO_FOUND)
    message(STATUS "[omniperception] Building with OpenVINO support")

    add_library(omniperception OBJECT
        decider.cpp
        perceptron.cpp
    )

    target_compile_definitions(omniperception PRIVATE ENABLE_OPENVINO)
    target_link_libraries(omniperception openvino::runtime)
else()
    message(STATUS "[omniperception] OpenVINO not found, skipping omniperception module")

    # 创建一个空的目标以避免链接错误
    add_library(omniperception INTERFACE)
endif()