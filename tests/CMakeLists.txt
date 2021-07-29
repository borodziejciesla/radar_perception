set(TEST_PROJECT_NAME radar_perception_tests)

project(${PROJECT_NAME} VERSION 0.1)

add_executable(${TEST_PROJECT_NAME}
    radar_processor_test.cpp
)

target_link_libraries(${TEST_PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
)