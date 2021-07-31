#include "dealiaser.hpp"

#include <ranges>

namespace measurements::radar
{
    Dealiaser::Dealiaser(void)
    {}

    Dealiaser::~Dealiaser(void)
    {}

    void Dealiaser::Run(RadarScan & radar_scan)
    {
        auto dealias = std::views::transform(
            [](RadarDetection & detection) {
                detection.range_rate = 0.0f;
            }
        );

        auto results = radar_scan.detections;// | dealias;
    }
}   // namespace measurements::radar
