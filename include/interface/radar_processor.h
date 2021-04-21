#ifndef RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_PROCESSOR_H_
#define RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_PROCESSOR_H_

#include "processor_calibration.h"
#include "radar_scan.h"

namespace measurements::radar
{
    class RadarProcessor
    {
        public:
            RadarProcessor(void);
            ~RadarProcessor(void);

            void Initialize(const ProcessorCalibration & calibration);

        private:
            ProcessorCalibration calibration_;
    };
}

#endif //RADAR_PERCEPTION_INCLUDE_INTERFACE_RADAR_PROCESSOR_H_
