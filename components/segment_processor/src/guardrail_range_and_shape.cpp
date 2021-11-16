/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "guardrail_range_and_shape.hpp"

#include <Eigen/Dense>

namespace measurements::radar
{
    void GuardrailRangeAndShape::operator()(const RadarDetection & detection) {
        // Range
        min_range_ = std::min(min_range_, detection.x);
        max_range_ = std::max(max_range_, detection.x);

        // Polynomial
        a0_ += 1.0f;
        a1_ += detection.x;
        a2_ += std::pow(detection.x, 2.0f);
        a3_ += std::pow(detection.x, 3.0f);
        a4_ += std::pow(detection.x, 4.0f);
        a5_ += std::pow(detection.x, 5.0f);
        a6_ += std::pow(detection.x, 6.0f);

        b1_ += detection.y;
        b2_ += detection.x * detection.y;
        b3_ += std::pow(detection.x, 2.0f) * detection.y;
        b4_ += std::pow(detection.x, 3.0f) * detection.y;

        // IDs
        ids_.push_back(detection.id);
    }

    Guardrail GuardrailRangeAndShape::GetGuardrail(void) {
        range_.start = min_range_;
        range_.end = max_range_;

        // Inverse matrix
        Eigen::MatrixXf a(4, 4);
        a(0, 0) = a0_;
        a(0, 1) = a(1, 0) = a1_;
        a(0, 2) = a(2, 0) = a2_;
        a(0, 3) = a(3, 0) = a3_;

        a(1, 1) = a2_;
        a(1, 2) = a(2, 1) = a3_;
        a(1, 3) = a(3, 1) = a4_;

        a(2, 2) = a4_;
        a(2, 3) = a(3, 2) = a5_;

        a(3, 3) = a6_;

        Eigen::MatrixXf b(4, 1);
        b(0) = b1_;
        b(1) = b2_;
        b(2) = b3_;
        b(3) = b4_;

        auto u = a.inverse() * b;

        guardrail_.range = range_;
        guardrail_.polynomial.a0 = u(0,0);
        guardrail_.polynomial.a1 = u(1,0);
        guardrail_.polynomial.a2 = u(2,0);
        guardrail_.polynomial.a3 = u(3,0);
        guardrail_.assigned_detdectios_ids = ids_;

        return guardrail_;
    }
}   //  namespace measurements::radar
