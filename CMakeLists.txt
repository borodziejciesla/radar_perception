cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(PROJECT_NAME radar_perception)
set(TEST_PROJECT_NAME radar_perception_tests)
project(${PROJECT_NAME})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_COMPILER /usr/bin/g++-10 CACHE PATH "")

set(DEFAULT_BUILD_TYPE "Debug")

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -g -O0 -fprofile-arcs -ftest-coverage")
endif()
if ( MSVC )
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /W4")
endif()

set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

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
include(cppcheck)
include(cpplint)

################ Unit Tests ###################
if(UNIX)
    # Testing
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIR})
    
    add_subdirectory(tests)
else()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        703bd9caab50b139428cea1aaff9974ebee5742e # release-1.10.0
    )

    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)
    add_subdirectory(tests)
endif()

# Code coverage
if (CMAKE_BUILD_TYPE STREQUAL "Coverage")
    include(CodeCoverage)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILER_FLAGS}")
    message(STATUS "Appending code coverage compiler flags: ${COVERAGE_COMPILER_FLAGS}")
    set(COVERAGE_EXCLUDES
		'tests/*'
		'${CMAKE_BINARY_DIR}/googletest-src/googletest/include/gtest/*'
		'${CMAKE_BINARY_DIR}/googletest-src/googletest/src/*'
		'${CMAKE_BINARY_DIR}/googletest-src/googlemock/include/gmock/*'
		'${CMAKE_BINARY_DIR}/googletest-src/googlemock/src/*'
    )
    setup_target_for_coverage(coverage ${TEST_PROJECT_NAME} ${PROJECT_NAME})

	set_target_properties(${PROJECT_NAME}
        PROPERTIES
        COMPILE_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage"
    )
endif() #CMAKE_BUILD_TYPE STREQUAL "Coverage"