/* Copyright (C) 2021 Maciej Rozewicz - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the XYZ license, which unfortunately won't be
 * written for another century.
 *
 * You should have received a copy of the XYZ license with
 * this file. If not, please write to: , or visit :
 */

#include "segment_processor.hpp"

namespace measurements::radar
{
    SegmentsProcessor::SegmentsProcessor(void) {
    }

    SegmentsProcessor::~SegmentsProcessor(void) {
    }

    std::tuple<MovingObjects, Guardrails> SegmentsProcessor::ProcessSegments(const RadarScan & radar_scan) {
    }
}   //  measurements::radar
