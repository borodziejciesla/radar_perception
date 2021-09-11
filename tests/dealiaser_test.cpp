#include "gtest/gtest.h"

#include <numeric>
#include <cmath>
#include <numbers>

#include "dealiaser.hpp"

class DealiaserTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}

        measurements::radar::DealiaserCalibration calibrations_;
};

TEST_F(DealiaserTests, ConstructorTest)
{
    std::unique_ptr<measurements::radar::Dealiaser> dealiaser;
    EXPECT_NO_THROW(dealiaser = std::make_unique<measurements::radar::Dealiaser>(calibrations_));
}

TEST_F(DealiaserTests, RunTest)
{
    measurements::radar::RadarScan scan;
    scan.detections.resize(10);
    measurements::radar::VelocityProfile vp;
    
    measurements::radar::Dealiaser dealiaser(calibrations_);
    dealiaser.Run(scan, vp);

    EXPECT_TRUE(true);
}

TEST_F(DealiaserTests, DealiasedTest)
{
    measurements::radar::RadarScan scan;
    scan.aliasing_period = 50.0f;
    measurements::radar::VelocityProfile vp;
    vp.vx = 10.0f;
    vp.vy = 0.0f;

    constexpr size_t detections_number = 64u;

    scan.detections.resize(detections_number);
    std::vector<float> azimuths(scan.detections.size());
    std::iota(azimuths.begin(), azimuths.end(), -30);
    std::transform(azimuths.begin(), azimuths.end(), azimuths.begin(), [](float degree) { return degree * std::numbers::pi / 180.0f; });
    
    std::transform(azimuths.begin(), azimuths.end(),
        scan.detections.begin(),
        [=](float azimuth) {
            static size_t id = 1;
            measurements::radar::RadarDetection detection;
            detection.id = id++;
            detection.range_rate = (vp.vx * std::cos(azimuth) + vp.vy * std::sin(azimuth));
            detection.range_rate_std = 0.1f;
            detection.azimuth = azimuth;
            detection.range = 10.0f;
            detection.x = detection.range * std::cos(detection.azimuth);
            detection.y = detection.range * std::sin(detection.azimuth);
            detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
            return detection;
        }
    );

    scan.detections.at(0).range_rate += scan.aliasing_period;
    scan.detections.at(1).range_rate -= scan.aliasing_period;
    
    calibrations_.dealiaser_threshold = 1.0f;

    measurements::radar::Dealiaser dealiaser(calibrations_);
    dealiaser.Run(scan, vp);

    EXPECT_FLOAT_EQ(scan.detections.at(0).range_rate, (vp.vx * std::cos(scan.detections.at(0).azimuth) + vp.vy * std::sin(scan.detections.at(0).azimuth)));
    EXPECT_FLOAT_EQ(scan.detections.at(1).range_rate, (vp.vx * std::cos(scan.detections.at(1).azimuth) + vp.vy * std::sin(scan.detections.at(1).azimuth)));
}
