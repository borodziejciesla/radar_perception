cmake_minimum_required(VERSION 3.16)

add_library(common
    src/range_rate.cpp
)

target_include_directories(common
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
