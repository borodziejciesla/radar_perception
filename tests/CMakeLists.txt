set(TEST_PROJECT_NAME radar_perception_tests)

project(${TEST_PROJECT_NAME} VERSION 0.1)

add_executable(${TEST_PROJECT_NAME}
    radar_processor_test.cpp
    dealiaser_test.cpp
    velocity_estimator_test.cpp

    $<TARGET_OBJECTS:radar_perception>
    $<TARGET_OBJECTS:dealiaser>
    $<TARGET_OBJECTS:detection_classifier>
    $<TARGET_OBJECTS:velocity_estimator>
)

target_link_libraries(${TEST_PROJECT_NAME}
    gtest_main
    GTest::GTest
    GTest::Main
    
    radar_perception
    dealiaser
    velocity_estimator
)