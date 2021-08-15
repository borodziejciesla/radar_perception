/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_MOVING_OBJECT_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_MOVING_OBJECT_HPP_

#include <vector>

#include "covariance.hpp"

namespace measurements::radar
{   
    struct Pose
    {
        float x = 0.0f;
        float y = 0.0f;
        float orientation = 0.0f;

        Covariance<3u> covariance;
    };

    struct Velocity
    {
        float vx = 0.0f;
        float vy = 0.0f;

        Covariance<2u> covariance;
    };

    struct Size
    {
        float length = 0.0f;
        float width = 0.0f;

        Covariance<2u> covariance;
    };

    struct MovingObject
    {
        Pose object_center;
        Velocity object_velocity;
        Size object_size;

        std::vector<size_t> assigned_detdectios_ids;
    };

    using MovingObjects = std::vector<MovingObject>;
}   // namespace measurements::radar


#endif  //  INCLUDE_RADAR_IO_INCLUDE_MOVING_OBJECT_HPP_
