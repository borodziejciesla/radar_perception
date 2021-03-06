/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_RADAR_VELOCITY_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_RADAR_VELOCITY_HPP_

#include <vector>

#include "covariance.hpp"

namespace measurements::radar
{
    struct RadarVelocity
    {
        std::array<float, 2u> velocity = {};
        Covariance<2u> covariance;
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_RADAR_VELOCITY_HPP_
