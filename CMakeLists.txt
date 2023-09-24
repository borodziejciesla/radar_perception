cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

set(PROJECT_NAME radar_perception)
set(TEST_PROJECT_NAME ${PROJECT_NAME}_tests)
set(EXAMPLE_PROJECT_NAME ${PROJECT_NAME}_example)

project(${PROJECT_NAME})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEFAULT_BUILD_TYPE "Debug")

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g -O0 -fprofile-arcs -ftest-coverage")
endif()
if ( MSVC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif()

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

option(BUILD_TESTS "Build tests" OFF)
option(BUILD_EXAMPLE "Build example" OFF)
option(MAKE_CPPCHECK "Make static code analysis" OFF)
option(MAKE_CPPLINT "Make code style analysis" OFF)

###############################################
########### Add sources and includes ##########
###############################################
include(GNUInstallDirs)

find_package(Eigen3 REQUIRED QUIET)

add_library(${PROJECT_NAME}
    src/radar_processor.cpp

    $<TARGET_OBJECTS:dealiaser>
    $<TARGET_OBJECTS:detection_classifier>
    $<TARGET_OBJECTS:segmentator>
    $<TARGET_OBJECTS:segment_processor>
    $<TARGET_OBJECTS:velocity_estimator>
)

add_subdirectory(include/radar_io include)
add_library(radar_io::radar_io ALIAS radar_io)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        radar_io
    PRIVATE
        Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include/
        $<INSTALL_INTERFACE:radar_io,include>
    PRIVATE
        $<TARGET_PROPERTY:dealiaser,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:detection_classifier,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:segmentator,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:segment_processor,INTERFACE_INCLUDE_DIRECTORIES>
        $<TARGET_PROPERTY:velocity_estimator,INTERFACE_INCLUDE_DIRECTORIES>
)

add_subdirectory(components)

###############################################
############### Static Analysis ###############
###############################################
include(cppcheck)
include(cpplint)

###############################################
############### CodeCoverage ##################
###############################################
if(UNIX)
    if (CMAKE_BUILD_TYPE STREQUAL "Coverage")
        include(CodeCoverage)

        SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
        
        setup_target_for_coverage(
            coverage
            ${TEST_PROJECT_NAME}
            ${PROJECT_NAME}_coverage_report
        ) 
    endif() #CMAKE_BUILD_TYPE STREQUAL "Coverage"
endif() # UNIX

###############################################
################# Examples ####################
###############################################
if(BUILD_EXAMPLE)
    project(${EXAMPLE_PROJECT_NAME})
    
    add_subdirectory(example)
endif() # BUILD_EXAMPLE

###############################################
################ Unit Tests ###################
###############################################
if(BUILD_TESTS)    
    project(${TEST_PROJECT_NAME})

    add_subdirectory(tests)    
    enable_testing()
    add_test(${TEST_PROJECT_NAME} ${TEST_PROJECT_NAME})    
endif() #   BUILD_TESTS
