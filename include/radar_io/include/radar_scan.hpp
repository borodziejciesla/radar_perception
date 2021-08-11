/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

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
