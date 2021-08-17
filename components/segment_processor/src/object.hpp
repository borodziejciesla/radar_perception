/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_SEGMENT_PROCESSOR_SRC_OBJECT_HPP_
#define COMPONENTS_SEGMENT_PROCESSOR_SRC_OBJECT_HPP_

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include "radar_detection.hpp"
#include "moving_object.hpp"

namespace measurements::radar
{
    class Object
    {
        public:
            void operator()(const RadarDetection & detection) {
                // Velocity
                square_c_ += std::pow(std::cos(detection.azimuth), 2.0f) / std::pow(detection.range_rate_std, 2.0f);
                square_s_ += std::pow(std::sin(detection.azimuth), 2.0f) / std::pow(detection.range_rate_std, 2.0f);
                cs_ += std::cos(detection.azimuth) * std::sin(detection.azimuth) / std::pow(detection.range_rate_std, 2.0f);

                b1_ += -std::cos(detection.azimuth) * detection.range_rate / std::pow(detection.range_rate_std, 2.0f);
                b2_ += -std::sin(detection.azimuth) * detection.range_rate / std::pow(detection.range_rate_std, 2.0f);

                // Position
                sum_x_ += detection.x;
                sum_y_ += detection.y;
                x_pose_cov_ += std::pow(detection.x_std, 2.0f);
                y_pose_cov_ += std::pow(detection.y_std, 2.0f);

                // Update ids
                ids_.push_back(detection.id);
            }

            MovingObject GetMovingObject(auto & segment) {
                EstimateVelocity(segment);
                EstimatePose(segment);
                EstimateSize(segment);

                return moving_object_;
            }

        private:
            void EstimateVelocity(auto & segment) {
                auto determinant = square_c_ * square_s_ - std::pow(cs_, 2.0f);
                
                auto ai11 = square_s_ / determinant;
                auto ai12 = -cs_ / determinant;
                auto ai22 = square_c_ / determinant;

                velocity_.vx = b1_ * ai11 + b2_ * ai12;
                velocity_.vy = b1_ * ai12 + b2_ * ai22;

                velocity_.covariance.covariance_diagonal.at(0) = ai11;
                velocity_.covariance.covariance_diagonal.at(1) = ai22;
                velocity_.covariance.covariance_lower_triangle.at(0) = ai12;

                moving_object_.object_velocity = velocity_;

                // Pose
                pose_.x = sum_x_ / static_cast<float>(ids_.size());
                pose_.y = sum_y_ / static_cast<float>(ids_.size());
                pose_.covariance.covariance_diagonal.at(0) = x_pose_cov_ / static_cast<float>(ids_.size());
                pose_.covariance.covariance_diagonal.at(1) = y_pose_cov_ / static_cast<float>(ids_.size());

                auto u_11 = 0.0f;
                auto u_20 = 0.0f;
                auto u_02 = 0.0f;
                auto u_11_cov = 0.0f;
                auto u_20_cov = 0.0f;
                auto u_02_cov = 0.0f;
                
                std::for_each(segment.begin(), segment.end(), [&](const RadarDetection & detection) {
                    auto delta_x = detection.x - pose_.x;
                    auto delta_y = detection.y - pose_.y;

                    u_11 += delta_x * delta_y;
                    u_11_cov += std::pow(delta_x * detection.x_std, 2.0f) + std::pow(delta_y * detection.y_std, 2.0f);
                    u_20 += std::pow(delta_x, 2.0f);
                    u_20_cov += std::pow(2.0f * detection.x * detection.x_std, 2.0f);
                    u_02 += std::pow(delta_y, 2.0f);
                    u_02_cov += std::pow(2.0f * detection.y * detection.y_std, 2.0f);
                });
                
                auto tan = 2.0f * u_11 / (u_20 - u_02);
                auto tan_diff_u_11_sqr = std::pow(2.0f / (u_20 - u_02), 2.0f);
                auto tan_diff_u_20_sqr = std::pow(2.0f * u_11 / std::pow(u_20 - u_02, 2.0f), 2.0f);
                auto tan_diff_u_02_sqr = std::pow(2.0f * u_11 / std::pow(u_20 - u_02, 2.0f), 2.0f);
                auto tan_cov = tan_diff_u_11_sqr * u_11_cov + tan_diff_u_20_sqr * u_20_cov + tan_diff_u_02_sqr * u_02_cov;

                auto orientation = 0.5f * std::atan(tan);
                auto diff_tan = 1.0f / (std::pow(tan, 2.0f) + 1.0f);
                auto orientation_cov = std::pow(diff_tan, 2.0f) * tan_cov;

                pose_.orientation = orientation;
                pose_.covariance.covariance_diagonal.at(2) = orientation_cov;

                moving_object_.object_center = pose_;
            }

            void EstimatePose(auto & segment) {
                pose_.x = sum_x_ / static_cast<float>(ids_.size());
                pose_.y = sum_y_ / static_cast<float>(ids_.size());
                pose_.covariance.covariance_diagonal.at(0) = x_pose_cov_ / static_cast<float>(ids_.size());
                pose_.covariance.covariance_diagonal.at(1) = y_pose_cov_ / static_cast<float>(ids_.size());

                auto u_11 = 0.0f;
                auto u_20 = 0.0f;
                auto u_02 = 0.0f;
                auto u_11_cov = 0.0f;
                auto u_20_cov = 0.0f;
                auto u_02_cov = 0.0f;
                
                std::for_each(segment.begin(), segment.end(), [&](const RadarDetection & detection) {
                    auto delta_x = detection.x - pose_.x;
                    auto delta_y = detection.y - pose_.y;

                    u_11 += delta_x * delta_y;
                    u_11_cov += std::pow(delta_x * detection.x_std, 2.0f) + std::pow(delta_y * detection.y_std, 2.0f);
                    u_20 += std::pow(delta_x, 2.0f);
                    u_20_cov += std::pow(2.0f * detection.x * detection.x_std, 2.0f);
                    u_02 += std::pow(delta_y, 2.0f);
                    u_02_cov += std::pow(2.0f * detection.y * detection.y_std, 2.0f);
                });
                
                auto tan = 2.0f * u_11 / (u_20 - u_02);
                auto tan_diff_u_11_sqr = std::pow(2.0f / (u_20 - u_02), 2.0f);
                auto tan_diff_u_20_sqr = std::pow(2.0f * u_11 / std::pow(u_20 - u_02, 2.0f), 2.0f);
                auto tan_diff_u_02_sqr = std::pow(2.0f * u_11 / std::pow(u_20 - u_02, 2.0f), 2.0f);
                auto tan_cov = tan_diff_u_11_sqr * u_11_cov + tan_diff_u_20_sqr * u_20_cov + tan_diff_u_02_sqr * u_02_cov;

                auto orientation = 0.5f * std::atan(tan);
                auto diff_tan = 1.0f / (std::pow(tan, 2.0f) + 1.0f);
                auto orientation_cov = std::pow(diff_tan, 2.0f) * tan_cov;

                pose_.orientation = orientation;
                pose_.covariance.covariance_diagonal.at(2) = orientation_cov;

                moving_object_.object_center = pose_;
            }

            void EstimateSize(auto & segment) {
                using Point = std::pair<float, float>;
                std::vector<Point> points_rotated(ids_.size());
                std::transform(segment.begin(), segment.end(), points_rotated.begin(),
                    [this](const RadarDetection & detection) {
                        auto delta_x = detection.x - pose_.x;
                        auto delta_y = detection.y - pose_.y;
                        auto c = std::cos(-detection.azimuth);
                        auto s = std::sin(-detection.azimuth);

                        auto x_rotated = delta_x * c - delta_y * s;
                        auto y_rotated = delta_x * s + delta_y * c;

                        return std::make_pair(x_rotated, y_rotated);
                    }
                );
                
                const auto [min_x, max_x] = std::minmax_element(points_rotated.begin(), points_rotated.end(),
                    [](const Point & a, const Point & b) {
                        return a.first < b.first;
                });
                const auto [min_y, max_y] = std::minmax_element(points_rotated.begin(), points_rotated.end(),
                    [](const Point & a, const Point & b) {
                        return a.second < b.second;
                });

                size_.length = max_x->first - min_x->first;
                size_.width = max_y->second - min_y->second;
                size_.covariance.covariance_diagonal = {1.0f, 1.0f};

                moving_object_.object_size = size_;
            }

            float square_c_ = 0.0f;
            float square_s_ = 0.0f;
            float cs_ = 0.0f;

            float b1_ = 0.0f;
            float b2_ = 0.0f;

            float sum_x_ = 0.0f;
            float sum_y_ = 0.0f;
            float x_pose_cov_ = 0.0f;
            float y_pose_cov_ = 0.0f;

            std::vector<size_t> ids_;
            Velocity velocity_;
            Pose pose_;
            Size size_;

            MovingObject moving_object_;
    };
}   //  namespace measurements::radar

#endif //   COMPONENTS_SEGMENT_PROCESSOR_SRC_OBJECT_HPP_
