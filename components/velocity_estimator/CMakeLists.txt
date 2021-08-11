cmake_minimum_required(VERSION 3.16)

add_library(velocity_estimator
    src/velocity_estimator.cpp

    $<TARGET_OBJECTS:common>
)

target_include_directories(velocity_estimator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>
)