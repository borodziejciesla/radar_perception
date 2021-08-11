#include "segmentator.hpp"

namespace measurements::radar
{
    Segmentator::Segmentator(const SegmentatorCalibration & calibration)
        : calibration_{calibration}{
    }

    Segmentator::~Segmentator(void) {
    }

    void Segmentator::Run(){}
}   //  namespace measurements::radar
