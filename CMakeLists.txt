cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(PROJECT_NAME radar_perception)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(
    include/common
    include/interface
)

add_library(${PROJECT_NAME}
    src/radar_processor.cpp)

### Unit Tests ###
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} GTest::GTest GTest::Main)

add_subdirectory(test)