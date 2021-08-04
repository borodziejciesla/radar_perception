#include "gtest/gtest.h"

#include <memory>

#include "radar_processor.hpp"

class RadarProcessorTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}

        measurements::radar::ProcessorCalibration calibrations_;
};

TEST_F(RadarProcessorTests, ConstructorTest)
{
    std::unique_ptr<measurements::radar::RadarProcessor> rp;
    EXPECT_NO_THROW(rp = std::make_unique<measurements::radar::RadarProcessor>(calibrations_));
}

TEST_F(RadarProcessorTests, RunTest)
{
    measurements::radar::RadarScan scan;
    scan.detections.resize(10);
    
    measurements::radar::RadarProcessor rp(calibrations_);
    rp.ProcessScan(scan);

    EXPECT_TRUE(true);
}
