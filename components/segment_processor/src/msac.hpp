/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_SEGMENT_PROCESSOR_SRC_MSAC_HPP_
#define COMPONENTS_SEGMENT_PROCESSOR_SRC_MSAC_HPP_

#include <algorithm>
#include <limits>
#include <vector>
#include <random>

#include "radar_scan.hpp"
#include "radar_velocity.hpp"

namespace measurements::radar
{
    using ObjectVelocity = RadarVelocity;

    class Msac
    {
        public:
            Msac(void) = default;
            Msac(const Msac & arg) = default;
            Msac(Msac && arg) = delete;
            ~Msac(void) = default;

            std::pair<ObjectVelocity, std::vector<size_t>> Run(RadarScan & radar_scan, const std::vector<size_t> segment_ids);

        private:
            std::pair<size_t, size_t> GetRandomPoints(const std::vector<size_t> segment_ids);
            std::pair<float, size_t> CalculateFit(const RadarScan & radar_scan, const std::vector<size_t> segment_ids);
            bool CalculateCurrentModel(const RadarScan & radar_scan, const size_t p1, const size_t p2);
            float CalculateSampleFit(const RadarDetection & detection);
            void SelectBestSolution(const float current_mean_error, const size_t current_inliers_number);
            void SetBestSolution(const float current_mean_error, const size_t current_inliers_number);
            std::pair<ObjectVelocity, std::vector<size_t>> CalculateFitBasedOnAllInliers(RadarScan & radar_scan);

            size_t maximum_iteration_number_ = 10u;
            size_t points_number_ = 0u;
            size_t best_inliers_number_ = 0u;
            float best_mean_error_ = std::numeric_limits<double>::infinity();
            std::default_random_engine rng_;
            float current_vx_ = 0.0f;
            float current_vy_ = 0.0f;
            float best_vx_ = 0.0f;
            float best_vy_ = 0.0f;
            std::vector<size_t> current_inliers_;
            std::vector<size_t> best_inliers_;
    };
}

#endif  //  COMPONENTS_SEGMENT_PROCESSOR_SRC_MSAC_HPP_
