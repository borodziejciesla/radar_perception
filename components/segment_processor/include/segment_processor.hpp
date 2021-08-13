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

#include "radar_scan.hpp"
#include "guardrail.hpp"
#include "moving_object.hpp"

namespace measurements::radar
{
    class SegmentsProcessor
    {
        public:
            SegmentsProcessor(void);
            ~SegmentsProcessor(void);

            std::tuple<MovingObjects, Guardrails> ProcessSegments(const RadarScan & radar_scan);
        private:
    };
}   //  namespace measurements::radar

#endif  //  COMPONENTS_SEGMENT_PROCESSOR_INCLUDE_SEGMENT_PROCESSOR_HPP_
