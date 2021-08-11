/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_SEGMENTATOR_CALIBRATIONS_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_SEGMENTATOR_CALIBRATIONS_HPP_

namespace measurements::radar
{
    struct SegmentatorCalibration
    {
        float neighbourhood_threshold = 0.0f;
        unsigned int minimum_detection_in_segment = 0u;
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_SEGMENTATOR_CALIBRATIONS_HPP_
