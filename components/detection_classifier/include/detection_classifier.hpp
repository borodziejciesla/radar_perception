/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DETECTION_CLASSIFIER_HPP_
#define COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DETECTION_CLASSIFIER_HPP_

#include "radar_scan.hpp"
#include "range_rate.hpp"

namespace measurements::radar
{
    class DetectionClassifier
    {
        public:
            DetectionClassifier(void);
            ~DetectionClassifier(void);

            void Run(RadarScan & radar_scan, const VelocityProfile & velocity_profile);

        private:
    };
}   // namespace measurements::radar

#endif // COMPONENTS_DETECTION_CLASSIFIER_INCLUDE_DETECTION_CLASSIFIER_HPP_
