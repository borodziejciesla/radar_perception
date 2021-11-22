/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "detection_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>

#include "math.hpp"
#include "range_rate.hpp"

namespace measurements::radar
{
    DetectionClassifier::DetectionClassifier(void) = default;

    DetectionClassifier::~DetectionClassifier(void) = default;

    void DetectionClassifier::Run(RadarScan & radar_scan, const VelocityProfile & velocity_profile) {
        auto is_valid_detection = std::ranges::views::filter([=](const RadarDetection & detection) {
            return (detection.id != 0u);
        });
        auto valid_detections_view = radar_scan.detections | is_valid_detection;

        std::transform(valid_detections_view.begin(), valid_detections_view.end(),
            valid_detections_view.begin(),
            [=](RadarDetection & detection) {
                Azimuth azimuth_eith_covariance;
                azimuth_eith_covariance.value.at(0u) = detection.azimuth;
                azimuth_eith_covariance.covariance.covariance_diagonal.at(0u) = std::pow(detection.azimuth_std, 2.0f);

                auto range_rate_model = RangeRate2D(azimuth_eith_covariance, velocity_profile);

                auto range_rate_abs_diff = std::abs(detection.range_rate - range_rate_model.value.at(0u));
                auto range_rate_diff_cov = std::pow(detection.range_rate_std, 2.0f) + range_rate_model.covariance.covariance_diagonal.at(0u);

                auto mahalanobis_distance = std::pow(range_rate_abs_diff, 2.0f) / range_rate_diff_cov;
                auto threshold = InverseChiSquareDistribution(0.95f, 1.0f);

                if (mahalanobis_distance <= threshold)
                    detection.moving_status = MovingStatus::Static;

                return detection;
            }
        );
    }
}   // namespace measurements::radar
