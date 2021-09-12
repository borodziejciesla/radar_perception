/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_
#define COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_

#include <vector>

#include "radar_scan.hpp"
#include "range_rate.hpp"
#include "dealiaser_calibrations.hpp"

namespace measurements::radar
{
    class Dealiaser
    {
        public:
            explicit Dealiaser(const DealiaserCalibration & dealiaser_calibrations);
            ~Dealiaser(void);

            void Run(RadarScan & radar_scan, const VelocityProfile & velocity_profile);

        private:
            const DealiaserCalibration calibrations_;
    };
}   // namespace measurements::radar

#endif // COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_
