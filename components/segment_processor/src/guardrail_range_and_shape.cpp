/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "guardrail_range_and_shape.hpp"

namespace measurements::radar
{
    void GuardrailRangeAndShape::operator()(const RadarDetection & detection) {
        // Range
        min_range_ = std::min(min_range_, detection.x);
        max_range_ = std::max(max_range_, detection.x);

        // Polynomial
        a11_ += std::pow(detection.x, 6.0f) / std::pow(detection.y_std, 2.0f);
        a21_ += std::pow(detection.x, 5.0f) / std::pow(detection.y_std, 2.0f);
        a22_ += std::pow(detection.x, 4.0f) / std::pow(detection.y_std, 2.0f);
        a31_ += std::pow(detection.x, 4.0f) / std::pow(detection.y_std, 2.0f);
        a32_ += std::pow(detection.x, 3.0f) / std::pow(detection.y_std, 2.0f);
        a33_ += std::pow(detection.x, 2.0f) / std::pow(detection.y_std, 2.0f);
        a41_ += std::pow(detection.x, 3.0f) / std::pow(detection.y_std, 2.0f);
        a42_ += std::pow(detection.x, 2.0f) / std::pow(detection.y_std, 2.0f);
        a43_ += std::pow(detection.x, 1.0f) / std::pow(detection.y_std, 2.0f);
        a44_ += 1.0f / std::pow(detection.y_std, 2.0f);

        b1_ += std::pow(detection.x, 3.0f) * detection.y / std::pow(detection.y_std, 2.0f);
        b2_ += std::pow(detection.x, 2.0f) * detection.y / std::pow(detection.y_std, 2.0f);
        b3_ += std::pow(detection.x, 1.0f) * detection.y / std::pow(detection.y_std, 2.0f);
        b4_ += detection.y / std::pow(detection.y_std, 2.0f);

        // IDs
        ids_.push_back(detection.id);
    }

    Guardrail GuardrailRangeAndShape::GetGuardrail(void) {
        // TODO inverse matrix
        range_.start = min_range_;
        range_.end = max_range_;

        guardrail_.range = range_;
        guardrail_.polynomial = polynomial_;
        guardrail_.assigned_detdectios_ids = ids_;

        return guardrail_;
    }
}   //  namespace measurements::radar
