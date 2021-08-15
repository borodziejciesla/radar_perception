/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_COMMON_INCLUDE_RANGE_RATE_HPP_
#define COMPONENTS_COMMON_INCLUDE_RANGE_RATE_HPP_

#include "velocity_profile.hpp"

namespace measurements::radar
{
    float RangeRate2D(const float azimuth, const VelocityProfile & velocity_prifle);
}   // measurements::radar

#endif  //  COMPONENTS_COMMON_INCLUDE_RANGE_RATE_HPP_
