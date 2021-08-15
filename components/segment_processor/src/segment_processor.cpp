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

namespace measurements::radar
{
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
        static auto mo = MovingObject();

        // Velocity Estimation
        struct ObjectVelocity {
            public:
                void operator()(const RadarDetection & detection) {
                    square_c += std::pow(std::cos(detection.azimuth), 2.0f) / std::pow(detection.range_rate_std, 2.0f);
                    square_s += std::pow(std::sin(detection.azimuth), 2.0f) / std::pow(detection.range_rate_std, 2.0f);
                    cs += std::cos(detection.azimuth) * std::sin(detection.azimuth) / std::pow(detection.range_rate_std, 2.0f);

                    b1 += -std::cos(detection.azimuth) * detection.range_rate / std::pow(detection.range_rate_std, 2.0f);
                    b2 += -std::sin(detection.azimuth) * detection.range_rate / std::pow(detection.range_rate_std, 2.0f);
                }

                Velocity GetVelocity(void) {
                    auto determinant = square_c * square_s - std::pow(cs, 2.0f);
                    
                    auto ai11 = square_s / determinant;
                    auto ai12 = -cs / determinant;
                    auto ai22 = square_c / determinant;

                    velocity.vx = b1 * ai11 + b2 * ai12;
                    velocity.vy = b1 * ai12 + b2 * ai22;

                    velocity.covariance.covariance_diagonal.at(0) = ai11;
                    velocity.covariance.covariance_diagonal.at(1) = ai22;
                    velocity.covariance.covariance_lower_triangle.at(0) = ai12;

                    return velocity;
                }

            private:
                float square_c = 0.0f;
                float square_s = 0.0f;
                float cs = 0.0f;

                float b1 = 0.0f;
                float b2 = 0.0f;

                Velocity velocity;
        };
        
        ObjectVelocity object_velocity = std::for_each(segment.begin(), segment.end(), ObjectVelocity());
        mo.object_velocity = object_velocity.GetVelocity();

        // Shape

        return mo;
    }

    Guardrail SegmentsProcessor::ProcessGuardrail(auto segment) {
        static auto g = Guardrail();
        
        // Barrier Shape
        struct GuardrailRangeAndShape {
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
                }

                std::tuple<Range, Polynomial> GetGuardrail(void) {
                    // TODO inverse matrix
                    range.start = min_range;
                    range.end = max_range;
                    return {range, polynomial};
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

                Range range;
                Polynomial polynomial;
        };

        GuardrailRangeAndShape guardrail_range_and_shape = std::for_each(segment.begin(), segment.end(), GuardrailRangeAndShape());
        auto [range, polynomial] = guardrail_range_and_shape.GetGuardrail();

        g.range = range;
        g.polynomial = polynomial;
        
        return g;
    }
}   //  measurements::radar
