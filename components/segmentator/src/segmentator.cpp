/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "segmentator.hpp"

namespace measurements::radar
{
    Segmentator::Segmentator(const SegmentatorCalibration & calibration)
        : calibration_{calibration} {
    }

    Segmentator::~Segmentator(void) {
    }

    void Segmentator::Run(RadarScan & radar_scan) {
    }

    void Segmentator::DbScan(RadarScan & radar_scan) {
    }

    int Segmentator::SelectInitialPoint(RadarScan & radar_scan) {
        return 0;
    }

    void Segmentator::FindAvailablePoints(int initial_point, RadarScan & radar_scan) {
    }
}   //  namespace measurements::radar
