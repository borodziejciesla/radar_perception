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
#include <ranges>
#include <cmath>

namespace measurements::radar
{
    Segmentator::Segmentator(const SegmentatorCalibration & calibration)
        : calibration_{calibration} {
    }

    Segmentator::~Segmentator(void) = default;

    void Segmentator::Run(RadarScan & radar_scan) {
        CalculateDistances(radar_scan);
        DbScan(radar_scan);   
    }

    void Segmentator::CalculateDistances(RadarScan & radar_scan) {
        distance_matrix_.SetSize(radar_scan.detections.size());
        
        for (auto first_index = 0u; first_index < radar_scan.detections.size(); first_index++) {
            for (auto second_index = 0u; second_index < first_index; second_index++) {
                distance_matrix_(first_index, second_index) = CalculateDistance(radar_scan.detections.at(first_index), radar_scan.detections.at(second_index));
            }
        }
    }

    void Segmentator::DbScan(RadarScan & radar_scan) {
        segmented_detections_number_ = 0u;
        current_segment_id_ = 1u;

        while (segmented_detections_number_ < radar_scan.detections.size()) {
            auto initial_point_index = SelectInitialPointIndex(radar_scan);
            if (!initial_point_index.has_value())
                break;
            FindAvailablePoints(initial_point_index.value(), radar_scan);
            current_segment_id_++;
        }
    }

    std::optional<size_t> Segmentator::SelectInitialPointIndex(RadarScan & radar_scan) {
        auto non_segmented_element = std::find_if(radar_scan.detections.begin(), radar_scan.detections.end(),
          [](const RadarDetection & detection) -> bool {
            return detection.segment_id == 0u;
          }  
        );

        if (non_segmented_element != radar_scan.detections.end())
            return non_segmented_element->id - 1u;
        else
           return std::nullopt;
    }

    void Segmentator::FindAvailablePoints(size_t initial_point_index, RadarScan & radar_scan) {
        auto is_segmented = std::ranges::views::filter([=](const RadarDetection & detection) {
            return (detection.segment_id == 0u) && (detection.id >= (initial_point_index + 1u));
        });
        auto non_segmented_view = radar_scan.detections | is_segmented;

        auto last_index = initial_point_index;
        std::transform(non_segmented_view.begin(), non_segmented_view.end(), non_segmented_view.begin(),
            [&,this](RadarDetection & detection) {
                if (distance_matrix_(last_index, detection.id - 1u) < calibration_.neighbourhood_threshold) {
                    detection.segment_id = current_segment_id_;
                    last_index = detection.id - 1u;
                    segmented_detections_number_++;
                }
                return detection;
            }
        );
    }

    float Segmentator::CalculateDistance(const RadarDetection & d1, const RadarDetection & d2) {
        return std::hypot(d1.x - d2.x, d1.y - d2.y);
    }
}   //  namespace measurements::radar
