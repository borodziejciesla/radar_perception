cmake_minimum_required(VERSION 3.16)

add_library(velocity_estimator
    src/velocity_estimator.cpp
)

target_include_directories(velocity_estimator
    PUBLIC
        include
)