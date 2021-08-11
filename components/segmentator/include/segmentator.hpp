/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_SEGMENTATOR_INCLUDE_SEGMENTATOR_HPP_
#define COMPONENTS_SEGMENTATOR_INCLUDE_SEGMENTATOR_HPP_

#include <optional>

#include "segmentator_calibration.hpp"
#include "radar_scan.hpp"

namespace measurements::radar
{
    class Segmentator
    {
        public:
            explicit Segmentator(const SegmentatorCalibration & calibration);
            ~Segmentator(void);

            void Run(RadarScan & radar_scan);

        private:
            void DbScan(RadarScan & radar_scan);
            std::optional<size_t> SelectInitialPointIndex(RadarScan & radar_scan);
            void FindAvailablePoints(int initial_point_index, RadarScan & radar_scan);

            SegmentatorCalibration calibration_;
            size_t segmented_detections_number = 0u;
    };
}   // namespace measurements::radar

#endif // COMPONENTS_SEGMENTATOR_INCLUDE_SEGMENTATOR_HPP_
