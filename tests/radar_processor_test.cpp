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

TEST_F(RadarProcessorTests, RunEmptyScanTest)
{
    measurements::radar::RadarScan scan;

    calibrations_.segmentator_calibration.minimum_detection_in_segment = 2u;
    calibrations_.segmentator_calibration.neighbourhood_threshold = 10.0f;
    
    measurements::radar::RadarProcessor rp(calibrations_);
    auto output = rp.ProcessScan(scan);

    EXPECT_FALSE(output.has_value());
}


TEST_F(RadarProcessorTests, RunTest)
{
    measurements::radar::RadarScan scan;
    for (auto index = 0u; index < 10u; index++) {
        measurements::radar::RadarDetection detection;
        detection.id = index + 1u;
        detection.x = 5.0f + static_cast<float>(index) * 0.5f;
        detection.x_std = 0.25f;
        detection.y = 5.0f;
        detection.y_std = 0.25f;
        detection.range = std::hypot(detection.x, detection.y);
        detection.azimuth = std::atan2(detection.y, detection.x);
        detection.range_rate = 0.0f;
        detection.range_rate_std = 0.01f;
        detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
        scan.detections.push_back(detection);
    }

    calibrations_.segmentator_calibration.minimum_detection_in_segment = 2u;
    calibrations_.segmentator_calibration.neighbourhood_threshold = 10.0f;
    
    measurements::radar::RadarProcessor rp(calibrations_);
    auto output = rp.ProcessScan(scan);

    EXPECT_TRUE(output.has_value());
}
