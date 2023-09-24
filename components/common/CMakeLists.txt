cmake_minimum_required(VERSION 3.16)

find_package(Eigen3 REQUIRED QUIET)

add_library(common
    src/range_rate.cpp
    src/math.cpp
)

target_link_libraries(common
    Eigen3::Eigen
)

target_include_directories(common
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
)
