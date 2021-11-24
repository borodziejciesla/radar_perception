#ifndef EXAMPLE_INCLUDE_DETECTION_HPP_
#define EXAMPLE_INCLUDE_DETECTION_HPP_

#include <cstdlib>
#include <vector>

struct Detection
{
    float range = 0.0f;
    float range_std = 0.0f;
    float azimuth = 0.0f;
    float azimuth_std = 0.0f;
    float range_rate = 0.0f;
    float range_rate_std = 0.0f;

    size_t object_id = 0u;
};

struct Scan {
    size_t scan_index = 0u;
    double time_stamp = 0.0;

    std::vector<Detection> detections;
};

#endif //   EXAMPLE_INCLUDE_DETECTION_HPP_
