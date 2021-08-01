/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_COMMON_INCLUDE_VELOCITY_PROFILE_HPP_
#define COMPONENTS_COMMON_INCLUDE_VELOCITY_PROFILE_HPP_

namespace measurements::radar
{
    struct VelocityProfile
    {
        float vx = 0.0f;
        float vy = 0.0f;
    };
}   // measurements::radar

#endif  //  COMPONENTS_COMMON_INCLUDE_VELOCITY_PROFILE_HPP_
