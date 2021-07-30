/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_PROCESSOR_H_
#define RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_PROCESSOR_H_

#include <memory>

#include "processor_calibration.hpp"
#include "radar_scan.hpp"

namespace measurements::radar
{
    class Dealiaser;
    class DetectionClassifier;
    class VelocityEstimator;

    class RadarProcessor
    {
        public:
            RadarProcessor(void);
            ~RadarProcessor(void);

            void Initialize(const ProcessorCalibration & calibration);
            void ProcessScan(RadarScan & radar_scan);

        private:
            ProcessorCalibration calibration_;
            std::unique_ptr<Dealiaser> dealiaser_;
            std::unique_ptr<DetectionClassifier> detection_classifier_;
            std::unique_ptr<VelocityEstimator> velocity_estimator_;
    };
}

#endif //RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_PROCESSOR_H_
