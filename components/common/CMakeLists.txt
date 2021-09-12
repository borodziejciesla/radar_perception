cmake_minimum_required(VERSION 3.16)

add_library(common
    src/range_rate.cpp
    src/math.cpp
)

target_include_directories(common
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:eigen,INTERFACE_INCLUDE_DIRECTORIES>
)
