/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_RADAR_DETECTION_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_RADAR_DETECTION_HPP_

#include "dealiasing_status.hpp"
#include "moving_status.hpp"

namespace measurements::radar
{
    struct RadarDetection
    {
        size_t id = 0u;

        float range = 0.0f;
        float azimuth = 0.0f;
        float elevaion = 0.0f;
        float range_rate = 0.0f;

        float range_std = 0.0f;
        float azimuth_std = 0.0f;
        float elevaion_std = 0.0f;
        float range_rate_std = 0.0f;

        DealiasingStatus dealiasing_status = DealiasingStatus::NonDealiased;
        MovingStatus moving_status = MovingStatus::Ambiguous;

        size_t segment_id = 0u;
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_RADAR_DETECTION_HPP_
