/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "range_rate.hpp"

#include <cmath>
#include <tuple>

namespace measurements::radar
{
    float RangeRate2D(const float azimuth, const VelocityProfile & velocity_prifle) {
        return (std::cos(azimuth) * velocity_prifle.vx) + (std::sin(azimuth) * velocity_prifle.vy);
    }

    float RangeRate3D(const float azimuth, const float elevation, const VelocityProfile & velocity_prifle) {
        std::ignore = azimuth;
        std::ignore = elevation;
        std::ignore = velocity_prifle;
        return 0.0f;
    }
}   //  namespace measurements::radar
