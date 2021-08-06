#include "gtest/gtest.h"

#include <memory>
#include <numeric>
#include <cmath>

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

    auto vx = 10.0f;
    auto vy = 0.5f;
    std::vector<float> azimuths(scan.detections.size());
    std::iota(azimuths.begin(), azimuths.end(), -5);
    
    std::transform(azimuths.begin(), azimuths.end(),
        scan.detections.begin(),
        [=](float azimuth) {
            measurements::radar::RadarDetection detection;
            detection.range_rate = vx * std::cos(azimuth) + vy * std::sin(azimuth);
            detection.range_rate_std = 0.1f;
            detection.azimuth = azimuth;
            detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
            return detection;
        }
    );
    
    measurements::radar::RadarProcessor rp(calibrations_);
    rp.ProcessScan(scan);

    EXPECT_TRUE(true);
}
