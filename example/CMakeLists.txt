add_subdirectory(matplot_lib)

add_executable(${EXAMPLE_PROJECT_NAME}
  example.cpp
  src/csv_reader.cpp
  src/sensors_data_reader.cpp
  src/detections_reader.cpp

  matplot_lib/matplotlibcpp.h

  $<TARGET_OBJECTS:radar_perception>
  $<TARGET_OBJECTS:dealiaser>
  $<TARGET_OBJECTS:detection_classifier>
  $<TARGET_OBJECTS:segmentator>
  $<TARGET_OBJECTS:segment_processor>
  $<TARGET_OBJECTS:velocity_estimator>
)

find_package(Python3 COMPONENTS Interpreter Development)
if (${Python3_FOUND})
  include_directories(${Python3_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Python3 not found, please install it.")
endif()

include_directories(${EXAMPLE_PROJECT_NAME}
  PUBLIC
    include
    radar_perception
    matplot_lib
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${EXAMPLE_PROJECT_NAME}
  radar_perception
  dealiaser
  detection_classifier
  segmentator
  segment_processor
  velocity_estimator
  ${Python3_LIBRARIES}
)