#include "gtest/gtest.h"

#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>

#include "velocity_estimator.hpp"
#include "velocity_estimator_calibrations.hpp"

class VelocityEstimatorTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}

        measurements::radar::VelocityEstimatorCalibration calibrations_;
};

TEST_F(VelocityEstimatorTests, ConstructorTest)
{
    std::unique_ptr<measurements::radar::VelocityEstimator> velocity_estimator;
    EXPECT_NO_THROW(velocity_estimator = std::make_unique<measurements::radar::VelocityEstimator>(calibrations_));
}

TEST_F(VelocityEstimatorTests, RunTest)
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
    
    measurements::radar::VelocityEstimator velocity_estimator = measurements::radar::VelocityEstimator(calibrations_);
    auto vp = velocity_estimator.Run(scan);

    EXPECT_NEAR(vp.vx, vx, 1e-4);
    EXPECT_NEAR(vp.vy, vy, 1e-4);
}
