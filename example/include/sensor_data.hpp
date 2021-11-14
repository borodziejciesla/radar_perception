#ifndef EXAMPLE_INCLUDE_SENSOR_DATA_HPP_
#define EXAMPLE_INCLUDE_SENSOR_DATA_HPP_

#include <cstdlib>

struct SensorData
{
    size_t sensor_index;
    
    float update_interval;
    
    float sensor_location_x;
    float sensor_location_y;
    float yaw;
    float pitch;
    float roll;
    
    float detection_probability;
    float azimuth_resolution;
    float range_resolution;
    float range_rate_resolution;
    
    size_t max_num_detections;
};

#endif //   EXAMPLE_INCLUDE_SENSOR_DATA_HPP_
