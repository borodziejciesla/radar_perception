cmake_minimum_required(VERSION 3.16)

add_library(segmentator
    src/segmentator.cpp

    $<TARGET_OBJECTS:common>
)

target_include_directories(segmentator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>
)