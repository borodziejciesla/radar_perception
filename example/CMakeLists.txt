#add_subdirectory(matplotlib_cpp)

add_executable(${EXAMPLE_PROJECT_NAME}
  example.cpp
  src/csv_reader.cpp
  src/sensors_data_reader.cpp
  src/detections_reader.cpp

  #$<TARGET_OBJECTS:radar_perception>
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
    #matplotlib_cpp
    ${Python3_INCLUDE_DIRS}
)

target_link_libraries(${EXAMPLE_PROJECT_NAME}
  #radar_perception
  ${Python3_LIBRARIES}
)