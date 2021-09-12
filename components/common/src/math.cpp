/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "math.hpp"

#include <cmath>

namespace measurements::radar
{
    float InverseChiSquareDistribution(const float x, const float df) {
        auto num = std::pow(2.0f, -0.5f * df);
        auto den = std::tgamma(0.5f * df);
        auto mul = std::pow(x, -0.5 * df - 1.0f) * std::exp(-1.0 / (2.0f * x));

        return num * mul / den;
    }
}   //  namespace measurements::radar
