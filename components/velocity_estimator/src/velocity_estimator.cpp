/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "velocity_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <ranges>

#include <Eigen/Dense>

#include "range_rate.hpp"

namespace measurements::radar
{
    VelocityEstimator::VelocityEstimator(const VelocityEstimatorCalibration & calibration)
        : calibration_{calibration} {
    }

    VelocityEstimator::~VelocityEstimator(void) = default;

    std::optional<VelocityProfile> VelocityEstimator::Run(const RadarScan & radar_scan) {
        best_iniliers_number_ = 0u;
        best_quality_ = std::numeric_limits<float>::infinity();

        for (auto index = 0u; index < calibration_.maximum_iterations_number; index++) {
            auto indices = GetRandomIndices(radar_scan);
            if (!indices.has_value())
                continue;
            auto [first, second] = indices.value();
            auto iteration_velocity_profile = FindIterationVelocity(radar_scan.detections.at(first), radar_scan.detections.at(second));
            auto [iniliers_number, fit_quality] = CalculateIniliersAndFitQuality(radar_scan, iteration_velocity_profile);

            if ( (iniliers_number > best_iniliers_number_) || ((iniliers_number == best_iniliers_number_) && (fit_quality < best_quality_)) ) {
                best_iniliers_number_ = iniliers_number;
                best_quality_ = fit_quality;
                best_velocity_profile_ = iteration_velocity_profile;
            }
        }

        if (best_iniliers_number_ == 0u)
            return std::nullopt;
        else
            return best_velocity_profile_;
    }

    std::optional<std::tuple<uint, uint>> VelocityEstimator::GetRandomIndices(const RadarScan & radar_scan) {
        std::vector<uint> indices = std::vector<uint>(radar_scan.detections.size());
        std::iota(indices.begin(), indices.end(), 0u);
        std::random_shuffle(indices.begin(), indices.end());

        auto check_if_dealiased = [=](const uint index) -> bool {
            return radar_scan.detections.at(index).dealiasing_status != DealiasingStatus::NonDealiased;
        };

        auto first_dealiased = std::find_if(indices.begin(), indices.end(), check_if_dealiased);
        if (first_dealiased == indices.end())
            return std::nullopt;
        auto second_dealiased = std::find_if(first_dealiased + 1, indices.end(), check_if_dealiased);
        if (second_dealiased == indices.end())
            return std::nullopt;

        return std::make_tuple(*first_dealiased, *second_dealiased);
    }

    const VelocityProfile & VelocityEstimator::FindIterationVelocity(const RadarDetection & first, const RadarDetection & second) {
        static VelocityProfile vp;

        auto s1 = std::sin(first.azimuth);
        auto c1 = std::cos(first.azimuth);
        auto r1 = first.range_rate;
        auto s2 = std::sin(second.azimuth);
        auto c2 = std::cos(second.azimuth);
        auto r2 = second.range_rate;

        /* Value */
        auto determinant = c1 * s2 - s1 * c2;
        vp.value.at(0u) = (s2 * r1 - s1 * r2) / determinant;
        vp.value.at(1u) = (-c2 * r1 + c1 * r2) / determinant;

        /* Covariance */
        Eigen::MatrixXf input_covariance = Eigen::MatrixXf::Zero(4, 4);
        input_covariance(0, 0) = first.azimuth_std;
        input_covariance(1, 1) = first.range_rate_std;
        input_covariance(2, 2) = second.azimuth_std;
        input_covariance(3, 3) = second.range_rate_std;

        Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(2, 4);
        jacobian(0, 0) = ((s1 * s2 + c1 * c2) * (r1 * s2 - r2 * s1)) / std::pow(c1 * s2 - c2 * s1, 2.0f) - (r2 * c1) / (c1 * s2 - c2 * s1);
        jacobian(0, 1) = s2 / (c1 * s2 - c2 * s1);
        jacobian(0, 2) = (r1 * c2) / (c1 * s2 - c2 * s1) - ((s1 * s2 + c1 * c2) * (r1 * s2 - r2 * s1)) / std::pow(c1 * s2 - c2 * s1, 2.0f);
        jacobian(0, 3) = -s1/(c1*s2 - c2*s1);
        jacobian(1, 0) = -(r2 * s1) / (c1 * s2 - c2 * s1) - ((s1 * s2 + c1 * c2) * (r1 * c2 - r2 * c1)) / std::pow(c1 * s2 - c2 * s1, 2.0f);
        jacobian(1, 1) = -c2 / (c1 * s2 - c2 * s1);
        jacobian(1, 2) = (r1 * s2) / (c1 * s2 - c2 * s1) + ((s1 * s2 + c1 * c2) * (r1 * c2 - r2 * c1)) / std::pow(c1 * s2 - c2 * s1, 2.0f);
        jacobian(1, 3) = c1 / (c1 * s2 - c2 * s1);

        auto covariance = jacobian * input_covariance * jacobian.transpose();

        vp.covariance.covariance_diagonal.at(0) = covariance(0, 0);
        vp.covariance.covariance_diagonal.at(1) = covariance(1, 1);
        vp.covariance.covariance_lower_triangle.at(0u) = covariance(0, 1);

        return vp;
    }

    std::tuple<uint, float> VelocityEstimator::CalculateIniliersAndFitQuality(const RadarScan & radar_scan, const VelocityProfile & velocity_profile) {
        auto iniliers_number = 0u;
        auto is_inlier = std::ranges::views::filter([this, velocity_profile, &iniliers_number](const RadarDetection & detection) {
            Azimuth azimuth_with_covariance;
            azimuth_with_covariance.value.at(0u) = detection.azimuth;
            azimuth_with_covariance.covariance.covariance_diagonal.at(0u) = detection.azimuth;

            auto model_range_rate = RangeRate2D(azimuth_with_covariance, velocity_profile);
            auto distance_from_model = std::abs(detection.range_rate - model_range_rate.value.at(0u));
            if (distance_from_model < calibration_.inlier_threshold) {
                iniliers_number++;
                return true;
            } else {
                return false;
            }
        });
        auto distance_to_profile = std::ranges::views::transform([=](const RadarDetection & detection) {
            Azimuth azimuth_with_covariance;
            azimuth_with_covariance.value.at(0u) = detection.azimuth;
            azimuth_with_covariance.covariance.covariance_diagonal.at(0u) = std::pow(detection.azimuth_std, 2.0f);

            auto model_range_rate = RangeRate2D(azimuth_with_covariance, velocity_profile);
            auto distance_from_model = std::abs(detection.range_rate - model_range_rate.value.at(0u));
            return std::pow(distance_from_model, 2.0f);
        });

        auto inliers_view = radar_scan.detections | is_inlier | distance_to_profile;
        auto fit_quality_sum = std::accumulate(inliers_view.begin(), inliers_view.end(), 0.0f);
        auto fit_quality_average = fit_quality_sum / static_cast<float>(iniliers_number);

        return std::make_tuple(iniliers_number, fit_quality_average);
    }
}   // namespace measurements::radar
