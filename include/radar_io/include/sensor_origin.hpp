/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_SENSOR_ORIGIN_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_SENSOR_ORIGIN_HPP_

namespace measurements::radar
{
    struct SensorOrigin
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float yaw = 0.0f;
        float pitch = 0.0f;
        float roll = 0.0f;
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_SENSOR_ORIGIN_HPP_
