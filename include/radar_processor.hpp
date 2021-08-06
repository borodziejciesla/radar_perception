/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_PROCESSOR_HPP_
#define INCLUDE_RADAR_PROCESSOR_HPP_

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
            explicit RadarProcessor(const ProcessorCalibration & calibration);
            ~RadarProcessor(void);

            void ProcessScan(RadarScan & radar_scan);

        private:
            ProcessorCalibration calibration_;
            std::unique_ptr<Dealiaser> dealiaser_;
            std::unique_ptr<DetectionClassifier> detection_classifier_;
            std::unique_ptr<VelocityEstimator> velocity_estimator_;
    };
}   // namespace measurements::radar

#endif  //  INCLUDE_RADAR_PROCESSOR_HPP_
