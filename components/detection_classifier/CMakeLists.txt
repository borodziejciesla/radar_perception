cmake_minimum_required(VERSION 3.16)

add_library(detection_classifier
    src/detection_classifier.cpp
)

target_include_directories(detection_classifier
    PUBLIC
        include
)
