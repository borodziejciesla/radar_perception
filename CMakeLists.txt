cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(PROJECT_NAME radar_perception)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTS ON)

########### Add sources and includes ###########
include(GNUInstallDirs)

add_library(${PROJECT_NAME}
    src/radar_processor.cpp

    $<TARGET_OBJECTS:dealiaser>
    $<TARGET_OBJECTS:detection_classifier>
    $<TARGET_OBJECTS:velocity_estimator>
)

add_subdirectory(include/io)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:radar_io, INTERFACE_INCLUDE_DIRECTORIES>
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include/
        include/io/include
    PRIVATE
        ${CMAKE_SOURCE_DIR}/components/dealiaser/include
        ${CMAKE_SOURCE_DIR}/components/detection_classifier/include
        ${CMAKE_SOURCE_DIR}/components/velocity_estimator/include
)

add_subdirectory(components)

############### Static Analysis ###############
include(cppcheck.cmake)

################ Unit Tests ###################
if (BUILD_TESTS)
    if(UNIX)
        # Testing
        enable_testing()
        find_package(GTest REQUIRED)
        include_directories(${GTEST_INCLUDE_DIR})
        
        target_link_libraries(${PROJECT_NAME} GTest::GTest GTest::Main)

        add_subdirectory(tests)
    endif()
endif()