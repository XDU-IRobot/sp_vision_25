cmake_minimum_required(VERSION 3.16)

add_library(tools STATIC
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
    float16.cpp
)

# 链接fmt库以解决undefined reference错误
target_link_libraries(tools PUBLIC fmt::fmt)