cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(PROJECT_NAME radar_perception)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

########### Add sources and includes ###########
add_subdirectory(components)

add_library(${PROJECT_NAME}
    src/radar_processor.cpp

    $<TARGET_OBJECTS:dealiaser>
    $<TARGET_OBJECTS:detection_classifier>
    $<TARGET_OBJECTS:velocity_estimator>
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include/
        include/io
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/components/dealiaser/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/components/detection_classifier/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/components/velocity_estimator/include>
)

############### Static Analysis ###############
include(cppcheck.cmake)

################ Unit Tests ###################
if(UNIX)
    # Testing
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIR})
    
    target_link_libraries(${PROJECT_NAME} GTest::GTest GTest::Main)

    add_subdirectory(tests)
endif()
