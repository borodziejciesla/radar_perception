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

#include "guardrail_range_and_shape.hpp"
#include "object.hpp"

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
        Object object = std::for_each(segment.begin(), segment.end(), Object());       
        return object.GetMovingObject(segment);
    }

    Guardrail SegmentsProcessor::ProcessGuardrail(auto segment) {        
        GuardrailRangeAndShape guardrail_range_and_shape = std::for_each(segment.begin(), segment.end(), GuardrailRangeAndShape());        
        return guardrail_range_and_shape.GetGuardrail();
    }
}   // namespace measurements::radar
