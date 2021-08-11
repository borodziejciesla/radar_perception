/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "segmentator.hpp"

#include <algorithm>

namespace measurements::radar
{
    Segmentator::Segmentator(const SegmentatorCalibration & calibration)
        : calibration_{calibration} {
    }

    Segmentator::~Segmentator(void) {
    }

    void Segmentator::Run(RadarScan & radar_scan) {
        DbScan(radar_scan);   
    }

    void Segmentator::DbScan(RadarScan & radar_scan) {
        while (segmented_detections_number < radar_scan.detections.size()) {
            auto initial_point_index = SelectInitialPointIndex(radar_scan);
            if (!initial_point_index.has_value())
                break;
            FindAvailablePoints(initial_point_index.value(), radar_scan);
        }
    }

    std::optional<size_t> Segmentator::SelectInitialPointIndex(RadarScan & radar_scan) {
        auto non_segmented_element = std::find_if(radar_scan.detections.begin(), radar_scan.detections.end(),
          [](const RadarDetection & detection) -> bool {
              detection.segment_id == 0u;
          }  
        );

        if (non_segmented_element != radar_scan.detections.end())
            return non_segmented_element->id - 1u;
        else
           return std::nullopt;
    }

    void Segmentator::FindAvailablePoints(int initial_point_index, RadarScan & radar_scan) {
    }
}   //  namespace measurements::radar
