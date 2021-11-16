/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_SEGMENT_PROCESSOR_SRC_GUARDRAIL_RANGE_AND_SHAPE_HPP_
#define COMPONENTS_SEGMENT_PROCESSOR_SRC_GUARDRAIL_RANGE_AND_SHAPE_HPP_

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include "radar_detection.hpp"
#include "guardrail.hpp"

namespace measurements::radar
{
    class GuardrailRangeAndShape
    {
        public:
            void operator()(const RadarDetection & detection);
            Guardrail GetGuardrail(void);

        private:
            float min_range_ = std::numeric_limits<float>::max();
            float max_range_ = std::numeric_limits<float>::lowest();

            float a0_ = 0.0f;
            float a1_ = 0.0f;
            float a2_ = 0.0f;
            float a3_ = 0.0f;
            float a4_ = 0.0f;
            float a5_ = 0.0f;
            float a6_ = 0.0f;

            float b1_ = 0.0f;
            float b2_ = 0.0f;
            float b3_ = 0.0f;
            float b4_ = 0.0f;

            std::vector<size_t> ids_;
            Range range_;
            Polynomial polynomial_;
            Guardrail guardrail_;
    };
}   //  namespace measurements::radar

#endif  //  COMPONENTS_SEGMENT_PROCESSOR_SRC_GUARDRAIL_RANGE_AND_SHAPE_HPP_
