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
#include <optional>

#include "guardrail.hpp"
#include "moving_object.hpp"
#include "processor_calibration.hpp"
#include "radar_scan.hpp"
#include "radar_velocity.hpp"

namespace measurements::radar
{
    class Dealiaser;
    class DetectionClassifier;
    class Segmentator;
    class SegmentsProcessor;
    class VelocityEstimator;

    class RadarProcessor
    {
        public:
            explicit RadarProcessor(const ProcessorCalibration & calibration);
            ~RadarProcessor(void);

            using ProcessingOutput = std::optional<std::tuple<MovingObjects, Guardrails>>;
            ProcessingOutput ProcessScan(RadarScan & radar_scan);
            const RadarVelocity & GetRadarVelocity(void) const;

        private:
            ProcessorCalibration calibration_;
            std::unique_ptr<Dealiaser> dealiaser_;
            std::unique_ptr<DetectionClassifier> detection_classifier_;
            std::unique_ptr<Segmentator> segmentator_;
            std::unique_ptr<SegmentsProcessor> segments_processor_;
            std::unique_ptr<VelocityEstimator> velocity_estimator_;
            RadarVelocity velocity_;
    };
}   // namespace measurements::radar

#endif  //  INCLUDE_RADAR_PROCESSOR_HPP_
