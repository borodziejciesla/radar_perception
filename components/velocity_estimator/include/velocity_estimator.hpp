/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_VELOCITY_ESTIMATOR_HPP_
#define COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_VELOCITY_ESTIMATOR_HPP_

#include "radar_scan.hpp"

#include <tuple>
#include <limits>
#include <vector>

#include "velocity_profile.hpp"

namespace measurements::radar
{
    class VelocityEstimator
    {
        public:
            VelocityEstimator(void);
            ~VelocityEstimator(void);

            const VelocityProfile & Run(const RadarScan & radar_scan);

        private:
            std::tuple<uint, uint> GetRandomIndices(size_t detections_number);
            VelocityProfile FindIterationVelocity(uint first, uint second);
            std::tuple<uint, float> CalculateIniliersAndFitQuality(const RadarScan & radar_scan, const VelocityProfile & velocity_profile);

            uint best_iniliers_number_ = 0u;
            float best_quality_ = std::numeric_limits<float>::infinity();
            VelocityProfile best_velocity_profile_;
    };
}   // namespace measurements::radar

#endif // COMPONENTS_VELOCITY_ESTIMATOR_INCLUDE_VELOCITY_ESTIMATOR_HPP_
