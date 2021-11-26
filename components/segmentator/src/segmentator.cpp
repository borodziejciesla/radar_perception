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

#include <Eigen/Dense>

#include "math.hpp"

namespace measurements::radar
{
    Segmentator::Segmentator(const SegmentatorCalibration & calibration)
        : calibration_{calibration}
        , threshold_{InverseChiSquareDistribution(calibration_.probability_hreshold, 2.0f)}  {
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
            return (detection.segment_id == 0u) && (detection.id >= (initial_point_index + 1u))
                && (detection.moving_status == radar_scan.detections.at(initial_point_index).moving_status);
        });
        auto non_segmented_view = radar_scan.detections | is_segmented;

        last_index_from_segment_ = initial_point_index;
        std::transform(non_segmented_view.begin(), non_segmented_view.end(), non_segmented_view.begin(),
            [&,this](RadarDetection & detection) {
                if (IsDetectionAssociated(detection)) {
                    detection.segment_id = current_segment_id_;
                    last_index_from_segment_ = detection.id - 1u;
                    segmented_detections_number_++;
                }
                return detection;
            }
        );
    }

    bool Segmentator::IsDetectionAssociated(const RadarDetection & detection) {
        return (distance_matrix_(last_index_from_segment_, detection.id - 1u) < threshold_);
    }

    float Segmentator::CalculateDistance(const RadarDetection & d1, const RadarDetection & d2) {
        static Eigen::Matrix2f covariance_d1, covariance_d2, covariance_diff;
        static Eigen::Vector2f diff;

        covariance_d1 << std::pow(d1.x_std, 2.0f) + 2.0f, 0.0f, 0.0f, std::pow(d1.y_std, 2.0f) + 2.0f;
        covariance_d2 << std::pow(d2.x_std, 2.0f) + 2.0f, 0.0f, 0.0f, std::pow(d2.y_std, 2.0f) + 2.0f;
        covariance_diff = covariance_d1 + covariance_d2;

        diff << d1.x - d2.x, d1.y - d2.y;
        
        auto mahalanobis_distance_matrix = diff.transpose() * covariance_diff.inverse() * diff;
        auto mahalanobis_distance = mahalanobis_distance_matrix(0);

        return mahalanobis_distance;
    }
}   //  namespace measurements::radar
