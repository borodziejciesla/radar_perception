cmake_minimum_required(VERSION 3.16)

find_package(Eigen3 REQUIRED QUIET)

add_library(velocity_estimator
    src/velocity_estimator.cpp

    $<TARGET_OBJECTS:common>
)

target_link_libraries(velocity_estimator
    Eigen3::Eigen
)

target_include_directories(velocity_estimator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>
)