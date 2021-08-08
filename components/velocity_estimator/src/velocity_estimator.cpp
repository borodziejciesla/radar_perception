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

#include "range_rate.hpp"

namespace measurements::radar
{
    VelocityEstimator::VelocityEstimator(const VelocityEstimatorCalibration & calibration)
        : calibration_{calibration} {
    }

    VelocityEstimator::~VelocityEstimator(void) {
    }

    const std::optional<VelocityProfile> & VelocityEstimator::Run(const RadarScan & radar_scan) {
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

        auto determinant = c1 * s2 - s1 * c2;
        auto vx = (s2 * r1 - s1 * r2) / determinant;
        auto vy = (-c2 * r1 + c1 * r2) / determinant;

        vp.vx = vx;
        vp.vy = vy;
        
        return vp;
    }

    std::tuple<uint, float> VelocityEstimator::CalculateIniliersAndFitQuality(const RadarScan & radar_scan, const VelocityProfile & velocity_profile) {
        auto dealiased_detections = std::ranges::find(radar_scan.detections, DealiasingStatus::NonDealiased, &RadarDetection::dealiasing_status);
                
        auto iniliers_number = 0u;
        auto accumulator_function = [velocity_profile,&iniliers_number](float accumulator, const RadarDetection & detection) -> float {
            auto model_range_rate = RangeRate2D(detection.azimuth, velocity_profile);
            auto distance_from_model = std::abs(detection.range_rate - model_range_rate);
            if (distance_from_model < 0.5f) {
                iniliers_number++;
                return accumulator + std::pow(distance_from_model, 2.0f);
            } else {
                return accumulator;
            }
        };
        
        //auto fit_quality_sum = std::accumulate(dealiased_detections.begin(), dealiased_detections.end(), 0.0f, accumulator_function)
        auto fit_quality_sum = std::accumulate(radar_scan.detections.begin(), radar_scan.detections.end(), 0.0f, accumulator_function);
        auto fit_quality_average = (iniliers_number == 0) ? 0.0f : fit_quality_sum / static_cast<float>(iniliers_number);

        return std::make_tuple(iniliers_number, fit_quality_average);
    }
}   // namespace measurements::radar
