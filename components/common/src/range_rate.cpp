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

#include <Eigen/Dense>

namespace measurements::radar
{
    const RangeRate & RangeRate2D(const Azimuth & azimuth, const VelocityProfile & velocity_prifle) {
        static RangeRate range_rate_output;

        auto ca = std::cos(azimuth.value.at(0u));
        auto sa = std::sin(azimuth.value.at(0u));

        auto vx = velocity_prifle.value.at(0u);
        auto vy = velocity_prifle.value.at(1u);

        Eigen::MatrixXf input_covariance = Eigen::MatrixXf::Zero(3, 3);
        input_covariance(0, 0) = velocity_prifle.covariance.covariance_diagonal.at(0u);
        input_covariance(1, 1) = velocity_prifle.covariance.covariance_diagonal.at(1u);
        input_covariance(1, 0) = velocity_prifle.covariance.covariance_lower_triangle.at(0u);
        input_covariance(0, 1) = input_covariance(1, 0);
        input_covariance(2, 2) = azimuth.covariance.covariance_diagonal.at(0u);

        Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(1, 3);
        jacobian(0, 0) = ca;
        jacobian(0, 1) = sa;
        jacobian(0, 2) = vy * ca - vy * sa;

        /* Range rate value */
        range_rate_output.value.at(0u) = ca * vx + sa * vy;

        /* Range rate covariance */
        auto covariance = jacobian * input_covariance * jacobian.transpose();
        range_rate_output.covariance.covariance_diagonal.at(0) = covariance(0, 0);

        return range_rate_output;
    }
}   //  namespace measurements::radar
