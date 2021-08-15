/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "segment_processor.hpp"

#include <ranges>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <utility>

namespace measurements::radar
{
    /************************* Moving Object *************************/
    struct Object
    {
        public:
            void operator()(const RadarDetection & detection) {
                // Velocity
                square_c += std::pow(std::cos(detection.azimuth), 2.0f) / std::pow(detection.range_rate_std, 2.0f);
                square_s += std::pow(std::sin(detection.azimuth), 2.0f) / std::pow(detection.range_rate_std, 2.0f);
                cs += std::cos(detection.azimuth) * std::sin(detection.azimuth) / std::pow(detection.range_rate_std, 2.0f);

                b1 += -std::cos(detection.azimuth) * detection.range_rate / std::pow(detection.range_rate_std, 2.0f);
                b2 += -std::sin(detection.azimuth) * detection.range_rate / std::pow(detection.range_rate_std, 2.0f);

                // Position
                sum_x += detection.x;
                sum_y += detection.y;
                x_pose_cov += std::pow(detection.x_std, 2.0f);
                y_pose_cov += std::pow(detection.y_std, 2.0f);

                // Update ids
                ids.push_back(detection.id);
            }

            MovingObject GetMovingObject(auto & segment) {
                // Velocity
                auto determinant = square_c * square_s - std::pow(cs, 2.0f);
                
                auto ai11 = square_s / determinant;
                auto ai12 = -cs / determinant;
                auto ai22 = square_c / determinant;

                velocity.vx = b1 * ai11 + b2 * ai12;
                velocity.vy = b1 * ai12 + b2 * ai22;

                velocity.covariance.covariance_diagonal.at(0) = ai11;
                velocity.covariance.covariance_diagonal.at(1) = ai22;
                velocity.covariance.covariance_lower_triangle.at(0) = ai12;

                moving_object.object_velocity = velocity;

                // Pose
                pose.x = sum_x / static_cast<float>(ids.size());
                pose.y = sum_y / static_cast<float>(ids.size());
                pose.covariance.covariance_diagonal.at(0) = x_pose_cov / static_cast<float>(ids.size());
                pose.covariance.covariance_diagonal.at(1) = y_pose_cov / static_cast<float>(ids.size());

                auto u_11 = 0.0f;
                auto u_20 = 0.0f;
                auto u_02 = 0.0f;
                auto u_11_cov = 0.0f;
                auto u_20_cov = 0.0f;
                auto u_02_cov = 0.0f;
                
                std::for_each(segment.begin(), segment.end(), [&](const RadarDetection & detection) {
                    auto delta_x = detection.x - pose.x;
                    auto delta_y = detection.y - pose.y;

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

                pose.orientation = orientation;
                pose.covariance.covariance_diagonal.at(2) = orientation_cov;

                moving_object.object_center = pose;

                // Size
                using Point = std::pair<float, float>;
                std::vector<Point> points_rotated(ids.size());
                std::transform(segment.begin(), segment.end(), points_rotated.begin(),
                    [this](const RadarDetection & detection) {
                        auto delta_x = detection.x - pose.x;
                        auto delta_y = detection.y - pose.y;
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

                size.length = max_x->first - min_x->first;
                size.width = max_y->second - min_y->second;
                size.covariance.covariance_diagonal = {1.0f, 1.0f};

                moving_object.object_size = size;

                // Output
                return moving_object;
            }

        private:
            float square_c = 0.0f;
            float square_s = 0.0f;
            float cs = 0.0f;

            float b1 = 0.0f;
            float b2 = 0.0f;

            float sum_x = 0.0f;
            float sum_y = 0.0f;
            float x_pose_cov = 0.0f;
            float y_pose_cov = 0.0f;

            std::vector<size_t> ids;
            Velocity velocity;
            Pose pose;
            Size size;

            MovingObject moving_object;
    };

    /************************* Static Object *************************/
    struct GuardrailRangeAndShape
    {
        public:
            void operator()(const RadarDetection & detection) {
                // Range
                min_range = std::min(min_range, detection.x);
                max_range = std::max(max_range, detection.x);

                // Polynomial
                a11 += std::pow(detection.x, 6.0f) / std::pow(detection.y_std, 2.0f);
                a21 += std::pow(detection.x, 5.0f) / std::pow(detection.y_std, 2.0f);
                a22 += std::pow(detection.x, 4.0f) / std::pow(detection.y_std, 2.0f);
                a31 += std::pow(detection.x, 4.0f) / std::pow(detection.y_std, 2.0f);
                a32 += std::pow(detection.x, 3.0f) / std::pow(detection.y_std, 2.0f);
                a33 += std::pow(detection.x, 2.0f) / std::pow(detection.y_std, 2.0f);
                a41 += std::pow(detection.x, 3.0f) / std::pow(detection.y_std, 2.0f);
                a42 += std::pow(detection.x, 2.0f) / std::pow(detection.y_std, 2.0f);
                a43 += std::pow(detection.x, 1.0f) / std::pow(detection.y_std, 2.0f);
                a44 += 1.0f / std::pow(detection.y_std, 2.0f);

                b1 += std::pow(detection.x, 3.0f) * detection.y / std::pow(detection.y_std, 2.0f);
                b2 += std::pow(detection.x, 2.0f) * detection.y / std::pow(detection.y_std, 2.0f);
                b3 += std::pow(detection.x, 1.0f) * detection.y / std::pow(detection.y_std, 2.0f);
                b4 += detection.y / std::pow(detection.y_std, 2.0f);

                // IDs
                ids.push_back(detection.id);
            }

            Guardrail GetGuardrail(void) {
                // TODO inverse matrix
                range.start = min_range;
                range.end = max_range;

                guardrail.range = range;
                guardrail.polynomial = polynomial;
                guardrail.assigned_detdectios_ids = ids;

                return guardrail;
            }

        private:
            float min_range = std::numeric_limits<float>::max();
            float max_range = std::numeric_limits<float>::lowest();

            float a11 = 0.0f;
            float a21 = 0.0f;
            float a22 = 0.0f;
            float a31 = 0.0f;
            float a32 = 0.0f;
            float a33 = 0.0f;
            float a41 = 0.0f;
            float a42 = 0.0f;
            float a43 = 0.0f;
            float a44 = 0.0f;

            float b1 = 0.0f;
            float b2 = 0.0f;
            float b3 = 0.0f;
            float b4 = 0.0f;

            std::vector<size_t> ids;
            Range range;
            Polynomial polynomial;
            Guardrail guardrail;
    };

    /************************* Segment Processor *************************/
    SegmentsProcessor::SegmentsProcessor(const SegmentProcessorCalibration & calibration)
        : calibration_{calibration} {
    }

    SegmentsProcessor::~SegmentsProcessor(void) {
    }

    std::tuple<MovingObjects, Guardrails> SegmentsProcessor::ProcessSegments(const RadarScan & radar_scan, const VelocityProfile & velocity_profile) {
        moving_objects_.clear();
        guardrails_.clear();
        
        /* Check for empty scan */
        if (radar_scan.detections.empty())
            return std::tuple{moving_objects_, guardrails_};

        /* Otherwise run processing */
        for (auto segment_id = 1u; segment_id < radar_scan.detections.size(); segment_id++)
        {
            auto segment = ProccessSegment(segment_id, radar_scan, velocity_profile);
            if (!segment.has_value())
                break;

            std::visit([this](auto arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, MovingObject>)
                    moving_objects_.push_back(arg);
                else
                    guardrails_.push_back(arg);
            }, segment.value());
        }

        return std::tuple{moving_objects_, guardrails_};
    }

    std::optional<SegmentsProcessor::Segment> SegmentsProcessor::ProccessSegment(size_t segment_id, const RadarScan & radar_scan, const VelocityProfile & velocity_profile) {
        auto selected_segment = std::ranges::views::filter([=](const RadarDetection & detection) {
            return detection.segment_id == segment_id;
        });

        auto segment_detections = radar_scan.detections | selected_segment;

        auto valid_detections_number = std::count_if(segment_detections.begin(), segment_detections.end(), [](const RadarDetection & detection) { return detection.id > 0u; });
        if (valid_detections_number == 0u)
            return std::nullopt;

        if (IsStaticSegment(segment_detections, velocity_profile))
            return ProcessGuardrail(segment_detections);
        else
            return ProcessMovingObject(segment_detections);
    }

    bool SegmentsProcessor::IsStaticSegment(auto segment, const VelocityProfile & velocity_profile) const {
        auto counter = 0u;
        auto error_sum = [velocity_profile,&counter](float error_sumed, const RadarDetection & detection) {
            auto abs_error = std::abs(detection.range_rate - (std::cos(detection.azimuth) * velocity_profile.vx + std::sin(detection.azimuth) * velocity_profile.vy));
            counter++;
            return error_sumed + abs_error;
        };

        auto summed_error = std::accumulate(segment.begin(), segment.end(), 0.0f, error_sum);
        auto average_error = summed_error / static_cast<float>(counter);

        return average_error < 1.0f;    // TODO : refactor
    }

    MovingObject SegmentsProcessor::ProcessMovingObject(auto segment) {
        Object object = std::for_each(segment.begin(), segment.end(), Object());       
        return object.GetMovingObject(segment);
    }

    Guardrail SegmentsProcessor::ProcessGuardrail(auto segment) {        
        GuardrailRangeAndShape guardrail_range_and_shape = std::for_each(segment.begin(), segment.end(), GuardrailRangeAndShape());        
        return guardrail_range_and_shape.GetGuardrail();
    }
}   // namespace measurements::radar
