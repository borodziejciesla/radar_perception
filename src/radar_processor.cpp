/* Copyright (C) 2021 MAciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "radar_processor.hpp"

namespace measurements::radar {
    RadarProcessor::RadarProcessor(void) {
        int l;
    }

    RadarProcessor::~RadarProcessor(void) {}

    void RadarProcessor::Initialize(const ProcessorCalibration & calibration) {
        calibration_ = calibration;
    }
}   // namespace measurements::radar
