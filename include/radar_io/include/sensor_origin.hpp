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
