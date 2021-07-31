cmake_minimum_required(VERSION 3.16)

add_library(dealiaser
    src/dealiaser.cpp
)

target_include_directories(dealiaser
    PUBLIC
        include
        ${CMAKE_SOURCE_DIR}/include/io/include
)
