/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "dealiaser.hpp"

#include <algorithm>
#include <cmath>

#include "range_rate.hpp"

namespace measurements::radar
{
    Dealiaser::Dealiaser(const DealiaserCalibration & dealiaser_calibrations)
        : calibrations_{dealiaser_calibrations} {
    }

    Dealiaser::~Dealiaser(void) = default;

    void Dealiaser::Run(RadarScan & radar_scan, const VelocityProfile & velocity_profile)
    {
        auto dealias = [=,this](RadarDetection & detection) {
            Azimuth azimuth_with_covariance;
            azimuth_with_covariance.value.at(0u) = detection.azimuth;
            azimuth_with_covariance.covariance.covariance_diagonal.at(0u) = std::pow(detection.azimuth_std, 2.0f);

            auto rr_model = RangeRate2D(azimuth_with_covariance, velocity_profile);
            auto rr_value = rr_model.value.at(0u);
            
            if (std::abs(rr_value - detection.range_rate) < calibrations_.dealiaser_threshold) {
                detection.dealiasing_status = DealiasingStatus::StaticVelocityProfileDealiased;
            } else if (std::abs((rr_value - radar_scan.aliasing_period) - detection.range_rate) < calibrations_.dealiaser_threshold) {
                detection.range_rate += radar_scan.aliasing_period;
                detection.dealiasing_status = DealiasingStatus::StaticVelocityProfileDealiased;
            } else if (std::abs((rr_value + radar_scan.aliasing_period) - detection.range_rate) < calibrations_.dealiaser_threshold) {
                detection.range_rate -= radar_scan.aliasing_period;
                detection.dealiasing_status = DealiasingStatus::StaticVelocityProfileDealiased;
            } else {
                // Do nothing - still non-dealiased
            }
        };

        std::for_each(radar_scan.detections.begin(), radar_scan.detections.end(), dealias);
    }
}   // namespace measurements::radar
