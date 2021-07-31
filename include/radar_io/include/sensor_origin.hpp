#ifndef RADAR_PERCEPTION_INCLUDE_INTERFACE_SENSOR_ORIGIN_H_
#define RADAR_PERCEPTION_INCLUDE_INTERFACE_SENSOR_ORIGIN_H_

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
}

#endif //RADAR_PERCEPTION_INCLUDE_INTERFACE_SENSOR_ORIGIN_H_
