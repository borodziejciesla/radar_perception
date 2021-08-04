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
#include "velocity_estimator.hpp"

namespace measurements::radar {
    RadarProcessor::RadarProcessor(const ProcessorCalibration & calibration)
        : calibration_{calibration}
        , dealiaser_{std::make_unique<Dealiaser>(calibration_.dealiaser_calibration)}
        , detection_classifier_{std::make_unique<DetectionClassifier>()}
        , velocity_estimator_{std::make_unique<VelocityEstimator>()} {
    }

    RadarProcessor::~RadarProcessor(void) {}

    void RadarProcessor::ProcessScan(RadarScan & radar_scan) {
        dealiaser_->Run(radar_scan);
        detection_classifier_->Run();
        velocity_estimator_->Run(radar_scan);
    }
}   // namespace measurements::radar
