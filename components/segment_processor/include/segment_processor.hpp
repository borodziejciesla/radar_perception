/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_SEGMENT_PROCESSOR_INCLUDE_SEGMENT_PROCESSOR_HPP_
#define COMPONENTS_SEGMENT_PROCESSOR_INCLUDE_SEGMENT_PROCESSOR_HPP_

#include <tuple>
#include <optional>
#include <variant>

#include "radar_scan.hpp"
#include "guardrail.hpp"
#include "moving_object.hpp"
#include "segment_processor_calibration.hpp"
#include "velocity_profile.hpp"

namespace measurements::radar
{
    class SegmentsProcessor
    {
        public:
            explicit SegmentsProcessor(const SegmentProcessorCalibration & calibration);
            ~SegmentsProcessor(void);

            std::tuple<MovingObjects, Guardrails> ProcessSegments(const RadarScan & radar_scan, const VelocityProfile & velocity_profile);

        private:
            using Segment = std::variant<MovingObject, Guardrail>;

            std::optional<Segment> ProccessSegment(size_t segment_id, const RadarScan & radar_scan, const VelocityProfile & velocity_profile);
            bool IsStaticSegment(auto segment, const VelocityProfile & velocity_profile) const;
            MovingObject ProcessMovingObject(auto segment);
            Guardrail ProcessGuardrail(auto segment);

            SegmentProcessorCalibration calibration_;
            MovingObjects moving_objects_;
            Guardrails guardrails_;
    };
}   //  namespace measurements::radar

#endif  //  COMPONENTS_SEGMENT_PROCESSOR_INCLUDE_SEGMENT_PROCESSOR_HPP_
