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
    float RangeRate2D(const float azimuth, const VelocityProfile & velocity_profile) {
        return (std::cos(azimuth) * velocity_profile.vx) + (std::sin(azimuth) * velocity_profile.vy);
    }
}   //  namespace measurements::radar
