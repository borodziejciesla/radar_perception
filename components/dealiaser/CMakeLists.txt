cmake_minimum_required(VERSION 3.16)

add_library(dealiaser
    src/dealiaser.cpp
)

target_include_directories(dealiaser
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        $<TARGET_PROPERTY:radar_io,INTERFACE_INCLUDE_DIRECTORIES>
)
