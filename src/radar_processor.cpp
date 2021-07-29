#include "radar_processor.hpp"

namespace measurements::radar
{
    RadarProcessor::RadarProcessor(void)
    {
        int l;
    }

    RadarProcessor::~RadarProcessor(void)
    {}

    void RadarProcessor::Initialize(const ProcessorCalibration & calibration)
    {
        calibration_ = calibration;
    }
}