cmake_minimum_required(VERSION 3.16)

add_library(segment_processor
    src/segment_processor.cpp
)

target_include_directories(segment_processor
    PUBLIC
        include
)
