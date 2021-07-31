cmake_minimum_required(VERSION 3.16)

project(radar_io VERSION 1.0.0)

add_library(radar_io INTERFACE)

target_include_directories(radar_io
    INTERFACE
        include
)
