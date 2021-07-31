#ifndef INCLUDE_RADAR_IO_INCLUDE_RADAR_DETECTION_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_RADAR_DETECTION_HPP_

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
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_RADAR_DETECTION_HPP_
