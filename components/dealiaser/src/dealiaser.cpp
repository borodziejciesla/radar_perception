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
        : calibrations_{dealiaser_calibrations}
    {}

    Dealiaser::~Dealiaser(void)
    {}

    void Dealiaser::Run(RadarScan & radar_scan, const VelocityProfile & velocity_profile)
    {
        auto dealias = [=,this](RadarDetection & detection) {
            auto rr_model = RangeRate2D(detection.azimuth, velocity_profile);
            
            if (std::abs(rr_model - detection.range_rate) < calibrations_.dealiaser_threshold)
            {
                detection.dealiasing_status = DealiasingStatus::StaticVelocityProfileDealiased;
            }
            else if (std::abs((rr_model - radar_scan.aliasing_period) - detection.range_rate) < calibrations_.dealiaser_threshold)
            {
                detection.range_rate += radar_scan.aliasing_period;
                detection.dealiasing_status = DealiasingStatus::StaticVelocityProfileDealiased;
            }
            else if (std::abs((rr_model + radar_scan.aliasing_period) - detection.range_rate) < calibrations_.dealiaser_threshold)
            {
                detection.range_rate -= radar_scan.aliasing_period;
                detection.dealiasing_status = DealiasingStatus::StaticVelocityProfileDealiased;
            }
            else
            {
                // Do nothing - still non-dealiased
            }
        };

        std::for_each(radar_scan.detections.begin(), radar_scan.detections.end(), dealias);
    }
}   // namespace measurements::radar
