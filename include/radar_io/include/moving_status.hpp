/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_MOVING_STATUS_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_MOVING_STATUS_HPP_

namespace measurements::radar
{
    enum class MovingStatus
    {
        Moving = 0u,
        Static = 1u,
        Ambiguous = 2u
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_MOVING_STATUS_HPP_
