/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_GUARDRAIL_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_GUARDRAIL_HPP_

#include <vector>

#include "covariance.hpp"

namespace measurements::radar
{
    struct Polynomial
    {
        float a0 = 0.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;
        float a3 = 0.0f;

        Covariance<4u> covariance;
    };

    struct Range
    {
        float start = 0.0f;
        float end = 0.0f;
    };

    struct Guardrail
    {
        Range range;
        Polynomial polynomial;

        std::vector<size_t> assigned_detdectios_ids;
    };

    using Guardrails = std::vector<Guardrail>;
}   // namespace measurements::radar


#endif  //  INCLUDE_RADAR_IO_INCLUDE_GUARDRAIL_HPP_
