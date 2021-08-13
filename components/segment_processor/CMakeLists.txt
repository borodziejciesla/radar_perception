cmake_minimum_required(VERSION 3.16)

add_library(segment_processor
    src/segment_processor.cpp
)

target_include_directories(segment_processor
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
)
