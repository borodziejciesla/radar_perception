/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_COVARIANCE_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_COVARIANCE_HPP_

#include <array>

namespace measurements::radar
{
    template <std::size_t size>
    struct Covariance
    {
        std::array<float, size> covariance_diagonal = {};
        std::array<float, (size) * (size - 1u) / 2u> covariance_lower_triangle = {};
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_COVARIANCE_HPP_
