/* Copyright (C) 2021 MAciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "radar_processor.hpp"

#include "dealiaser.hpp"
#include "detection_classifier.hpp"
#include "segmentator.hpp"
#include "segment_processor.hpp"
#include "velocity_estimator.hpp"

namespace measurements::radar {
    RadarProcessor::RadarProcessor(const ProcessorCalibration & calibration)
        : calibration_{calibration}
        , dealiaser_{std::make_unique<Dealiaser>(calibration_.dealiaser_calibration)}
        , detection_classifier_{std::make_unique<DetectionClassifier>()}
        , segmentator_{std::make_unique<Segmentator>(calibration_.segmentator_calibration)}
        , segments_processor_{std::make_unique<SegmentsProcessor>(calibration_.segment_processor_calibration_)}
        , velocity_estimator_{std::make_unique<VelocityEstimator>(calibration.velocity_estimator_calibration)} {
    }

    RadarProcessor::~RadarProcessor(void) {}

    RadarProcessor::ProcessingOutput RadarProcessor::ProcessScan(RadarScan & radar_scan) {
        auto velocity_profile = velocity_estimator_->Run(radar_scan);
        
        if (!velocity_profile.has_value())
            return std::nullopt;

        dealiaser_->Run(radar_scan, velocity_profile.value());
        detection_classifier_->Run();
        segmentator_->Run(radar_scan);
        return segments_processor_->ProcessSegments(radar_scan, velocity_profile.value());
    }
}   // namespace measurements::radar
