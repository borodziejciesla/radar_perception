#ifndef RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_DETECTION_H_
#define RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_DETECTION_H_

namespace measurements::radar
{
    struct RadarDetection
    {
        float range = 0.0f;
        float azimuth = 0.0f;
        float elevaion = 0.0f;
        float range_rate = 0.0f;

        float range_std = 0.0f;
        float azimuth_std = 0.0f;
        float elevaion_std = 0.0f;
        float range_rate_std = 0.0f;
    };
}

#endif //RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_DETECTION_H_
