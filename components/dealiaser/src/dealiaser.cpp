/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "dealiaser.hpp"

#include <ranges>
#include <algorithm>

namespace measurements::radar
{
    Dealiaser::Dealiaser(void)
    {}

    Dealiaser::~Dealiaser(void)
    {}

    void Dealiaser::Run(RadarScan & radar_scan)
    {
        auto dealias = [](RadarDetection & detection) {
            detection.range_rate = 0.0;
        };

        std::for_each(radar_scan.detections.begin(), radar_scan.detections.end(), dealias);
    }
}   // namespace measurements::radar
