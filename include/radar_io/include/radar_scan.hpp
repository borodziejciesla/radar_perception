#ifndef INCLUDE_RADAR_IO_INCLUDE_RADAR_SCAN_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_RADAR_SCAN_HPP_

#include <vector>

#include "radar_detection.hpp"
#include "sensor_origin.hpp"

namespace measurements::radar
{
    struct RadarScan
    {
        SensorOrigin sensor_origin;
        float aliasing_period = 0.0f;

        std::vector<RadarDetection> detections;
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_RADAR_SCAN_HPP_
