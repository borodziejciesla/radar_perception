/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_COMMON_INCLUDE_VALUE_WITH_COVARIANCE_HPP_
#define COMPONENTS_COMMON_INCLUDE_VALUE_WITH_COVARIANCE_HPP_

#include "covariance.hpp"

namespace measurements::radar
{
    template <std::size_t size>
    struct ValueWithCovariance
    {
        std::array<float, size> value;
        Covariance<size> covariance;
    };
}   // measurements::radar

#endif  //  COMPONENTS_COMMON_INCLUDE_VALUE_WITH_COVARIANCE_HPP_
