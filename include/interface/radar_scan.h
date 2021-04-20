#ifndef RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_SCAN_H_
#define RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_SCAN_H_

#include <vector>

#include "radar_detection.h"
#include "sensor_origin.h"

namespace measurements::radar
{
    struct RadarScan
    {
        SensorOrigin sensor_origin;
        std::vector<RadarDetection> detections;
    };
}

#endif //RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_SCAN_H_
