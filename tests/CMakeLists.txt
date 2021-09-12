################################
# GTest
################################
project(googletest-git NONE)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        703bd9caab50b139428cea1aaff9974ebee5742e # release-1.10.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

################################
# Tests
################################
add_executable(${TEST_PROJECT_NAME}
    radar_processor_test.cpp
    dealiaser_test.cpp
    detection_classifier_test.cpp
    segmentator_test.cpp
    segments_processor_test.cpp
    velocity_estimator_test.cpp

    $<TARGET_OBJECTS:radar_perception>
    $<TARGET_OBJECTS:dealiaser>
    $<TARGET_OBJECTS:detection_classifier>
    $<TARGET_OBJECTS:segmentator>
    $<TARGET_OBJECTS:segment_processor>
    $<TARGET_OBJECTS:velocity_estimator>
)

target_link_libraries(${TEST_PROJECT_NAME}
    gtest 
    gtest_main
        
    radar_perception
    dealiaser
    detection_classifier
    segmentator
    segment_processor
    velocity_estimator
)