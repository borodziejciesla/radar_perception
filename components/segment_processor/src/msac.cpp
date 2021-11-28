/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "msac.hpp"

#include <cmath>
#include <numeric>

#include <Eigen/Dense>

namespace measurements::radar
{
    std::pair<ObjectVelocity, std::vector<size_t>> Msac::Run(RadarScan & radar_scan, const std::vector<size_t> segment_ids) {
        points_number_ = segment_ids.size();

        for (auto iteration_index = 0u; iteration_index < maximum_iteration_number_; iteration_index++) {
            auto [p1, p2] = GetRandomPoints(segment_ids);
            auto is_current_model_valid = CalculateCurrentModel(radar_scan, p1, p2);
            
            if (!is_current_model_valid)
                continue;
            
            auto [mean_error, inliers_number] = CalculateFit(radar_scan, segment_ids);
            SelectBestSolution(mean_error, inliers_number);
        }

        return CalculateFitBasedOnAllInliers(radar_scan);
    }

    std::pair<size_t, size_t> Msac::GetRandomPoints(const std::vector<size_t> segment_ids) {
        std::vector<size_t> indices(points_number_);
        std::iota(indices.begin(), indices.end(), 0u);
        std::shuffle(indices.begin(), indices.end(), rng_);

        return std::pair<size_t, size_t>(segment_ids.at(indices.at(0u)), segment_ids.at(indices.at(1u)));
    }

    std::pair<float, size_t> Msac::CalculateFit(const RadarScan & radar_scan, const std::vector<size_t> segment_ids) {
        auto inliers_number = 0u;
        auto summary_error = 0.0f;
        current_inliers_.clear();
        
        for (const auto & detection_idx : segment_ids) {
            auto error = CalculateSampleFit(radar_scan.detections.at(detection_idx - 1u));
            if (error < 0.5f) {
                summary_error += error;
                inliers_number++;
                current_inliers_.push_back(detection_idx);
            }
        }

        auto average_error = summary_error / static_cast<float>(inliers_number);

        return std::pair<float, size_t>(average_error, inliers_number);
    }

    bool Msac::CalculateCurrentModel(const RadarScan & radar_scan, const size_t p1, const size_t p2) {
        auto azimuth_1 = radar_scan.detections.at(p1 - 1u).azimuth;
        auto azimuth_2 = radar_scan.detections.at(p2 - 1u).azimuth;
        auto range_rate_1 = radar_scan.detections.at(p1 - 1u).range_rate;
        auto range_rate_2 = radar_scan.detections.at(p2 - 1u).range_rate;

        auto c1 = std::cos(azimuth_1);
        auto s1 = std::sin(azimuth_1);
        auto c2 = std::cos(azimuth_2);
        auto s2 = std::sin(azimuth_2);

        auto determinant = c1 * s2 - s1 * c2;

        if (std::abs(determinant) < 1e-3f)
            return false;

        current_vx_ = (s2 * range_rate_1 - s1 * range_rate_2) / determinant;
        current_vy_ = (-c2 * range_rate_1 + c1 * range_rate_2) / determinant;
        return true;
    }

    float Msac::CalculateSampleFit(const RadarDetection & detection) {
        auto azimuth = detection.azimuth;
        auto range_rate = detection.range_rate;

        auto range_rate_model = current_vx_ * std::cos(azimuth) + current_vy_ * std::sin(azimuth);

        return std::abs(range_rate_model - range_rate);
    }

    void Msac::SelectBestSolution(const float current_mean_error, const size_t current_inliers_number) {
        auto is_new_best_inliers_number = current_inliers_number > best_inliers_number_;
        auto is_new_best_mean_error = (current_inliers_number == best_inliers_number_) && (best_mean_error_ < current_inliers_number);
        auto is_new_best = is_new_best_inliers_number || is_new_best_mean_error;
        
        if (is_new_best)
            SetBestSolution(current_mean_error, current_inliers_number);
    }

    void Msac::SetBestSolution(const float current_mean_error, const size_t current_inliers_number) {
        best_mean_error_ = current_mean_error;
        best_inliers_number_ = current_inliers_number;

        best_vx_ = current_vx_;
        best_vy_ = current_vy_;
        best_inliers_ = current_inliers_;
    }

    std::pair<ObjectVelocity, std::vector<size_t>> Msac::CalculateFitBasedOnAllInliers(RadarScan & radar_scan) {
        struct VelocityEstimator {
            public:
                VelocityEstimator(RadarScan & radar_scan) : radar_scan_{radar_scan} {
                }

                void operator()(const size_t detection_index) {
                    auto azimuth = radar_scan_.detections.at(detection_index - 1u).azimuth;
                    auto range_rate = radar_scan_.detections.at(detection_index - 1u).range_rate;
                    auto range_rate_std = radar_scan_.detections.at(detection_index - 1u).range_rate_std;

                    radar_scan_.detections.at(detection_index - 1u).moving_status = MovingStatus::Moving;

                    auto c = std::cos(azimuth);
                    auto s = std::sin(azimuth);

                    a_11 += std::pow(c, 2.0f);
                    a_12 += c * s;
                    a_22 += std::pow(s, 2.0f);

                    b_1 += range_rate * c;
                    b_2 += range_rate * s;

                    r_11 += std::pow(c * range_rate_std, 2.0f);
                    r_11 += s * c * std::pow(range_rate_std, 2.0f);
                    r_22 += std::pow(s * range_rate_std, 2.0f);
                }

                ObjectVelocity CalculateVelocity(void) {
                    Eigen::Matrix2f a;
                    a << a_11, a_12, a_12, a_22;

                    Eigen::Vector2f b;
                    b << b_1, b_2;

                    Eigen::Matrix2f r;
                    r << r_11, r_12, r_12, r_22;

                    auto a_inverse = a.inverse();

                    auto v = a_inverse * b;
                    auto covariance = a_inverse * r * a_inverse;

                    RadarVelocity velocity_with_covariance;
                    velocity_with_covariance.velocity = { v(0u), v(1u) };
                    velocity_with_covariance.covariance.covariance_diagonal = { covariance(0u, 0u), covariance(1u, 1u) };
                    velocity_with_covariance.covariance.covariance_lower_triangle = { covariance(0u, 1u) };

                    return velocity_with_covariance;
                }

            private:
                float a_11 = 0.0f;
                float a_12 = 0.0f;
                float a_22 = 0.0f;

                float b_1 = 0.0f;
                float b_2 = 0.0f;

                float r_11 = 0.0f;
                float r_12 = 0.0f;
                float r_22 = 0.0f;

                RadarScan & radar_scan_;
        };

        VelocityEstimator velocity_estimator = std::for_each(best_inliers_.begin(), best_inliers_.end(), VelocityEstimator(radar_scan));
        auto velocity = velocity_estimator.CalculateVelocity();

        return std::pair<ObjectVelocity, std::vector<size_t>>(velocity, best_inliers_);
    }
}   // namespace measurements::radar
