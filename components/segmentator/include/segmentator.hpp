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
#include <memory>

#include "segmentator_calibration.hpp"
#include "radar_scan.hpp"
#include "distance_matrix.hpp"

namespace measurements::radar
{
    class Segmentator
    {
        public:
            explicit Segmentator(const SegmentatorCalibration & calibration);
            ~Segmentator(void);

            void Run(RadarScan & radar_scan);

        private:
            void CalculateDistances(RadarScan & radar_scan);
            void DbScan(RadarScan & radar_scan);
            std::optional<size_t> SelectInitialPointIndex(RadarScan & radar_scan);
            void FindAvailablePoints(size_t initial_point_index, RadarScan & radar_scan);
            bool IsDetectionAssociated(const RadarDetection & detection);

            static float CalculateDistance(const RadarDetection & d1, const RadarDetection & d2);

            SegmentatorCalibration calibration_;
            DistanceMatrix distance_matrix_;
            size_t segmented_detections_number_ = 0u;
            size_t current_segment_id_ = 0u;
            float threshold_ = 0.0f;
            size_t last_index_from_segment_ = 0u;
    };
}   // namespace measurements::radar

#endif // COMPONENTS_SEGMENTATOR_INCLUDE_SEGMENTATOR_HPP_
