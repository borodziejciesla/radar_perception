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
#include "velocity_profile.hpp"

namespace measurements::radar
{
    class Dealiaser
    {
        public:
            Dealiaser(void);
            ~Dealiaser(void);

            void Run(RadarScan & radar_scan);

        private:
            VelocityProfile velocity_profile_;
    };
}   // namespace measurements::radar

#endif // COMPONENTS_DEALIASER_INCLUDE_DEALIASER_HPP_
