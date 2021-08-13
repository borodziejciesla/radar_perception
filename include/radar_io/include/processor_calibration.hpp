/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#ifndef INCLUDE_RADAR_IO_INCLUDE_PROCESSOR_CALIBRATION_HPP_
#define INCLUDE_RADAR_IO_INCLUDE_PROCESSOR_CALIBRATION_HPP_

#include "dealiaser_calibrations.hpp"
#include "segmentator_calibration.hpp"
#include "segment_processor_calibration.hpp"
#include "velocity_estimator_calibrations.hpp"

namespace measurements::radar
{
    struct ProcessorCalibration
    {
        DealiaserCalibration dealiaser_calibration;
        SegmentatorCalibration segmentator_calibration;
        SegmentProcessorCalibration segment_processor_calibration_;
        VelocityEstimatorCalibration velocity_estimator_calibration;
    };
}   // namespace measurements::radar

#endif // INCLUDE_RADAR_IO_INCLUDE_PROCESSOR_CALIBRATION_HPP_
