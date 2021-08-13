cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(PROJECT_NAME radar_perception)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_COMPILER /usr/bin/g++-10 CACHE PATH "")

set(DEFAULT_BUILD_TYPE "Debug")

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()
if ( MSVC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif()

########### Add sources and includes ###########
include(GNUInstallDirs)

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

############### Static Analysis ###############
include(cppcheck.cmake)
include(cpplint.cmake)

################ Unit Tests ###################
if(UNIX)
    # Testing
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIR})
    
    add_subdirectory(tests)
endif()
