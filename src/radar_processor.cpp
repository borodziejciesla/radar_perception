#include "radar_processor.h"

namespace measurements::radar
{
    RadarProcessor::RadarProcessor(void)
    {}

    RadarProcessor::~RadarProcessor(void)
    {}

    void RadarProcessor::Initialize(const ProcessorCalibration & calibration)
    {
        calibration_ = calibration;
    }
}