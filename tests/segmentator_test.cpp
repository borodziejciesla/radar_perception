#include "gtest/gtest.h"

#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>
#include <memory>
#include <numbers>

#include "segmentator_calibration.hpp"
#include "segmentator.hpp"

class SegmentatorTests : public ::testing::Test
{
    protected:
        void SetUp(void) override {
        }

        void TearDown(void) override {
        }

        measurements::radar::SegmentatorCalibration calibration_;
};

TEST_F(SegmentatorTests, ConstructorTest) {
    EXPECT_NO_THROW(auto segmentator = std::make_unique<measurements::radar::Segmentator>(calibration_));
}

TEST_F(SegmentatorTests, RunAllDetectionInSingleSegmentTest) {
    measurements::radar::RadarScan scan;
    scan.detections.resize(10);
    auto vx = 10.0f;
    auto vy = 0.5f;
    std::vector<float> azimuths(scan.detections.size());
    std::iota(azimuths.begin(), azimuths.end(), -5);
    
    std::transform(azimuths.begin(), azimuths.end(),
        scan.detections.begin(),
        [=](float azimuth) {
            static size_t id = 1;
            measurements::radar::RadarDetection detection;
            detection.id = id++;
            detection.range_rate = vx * std::cos(azimuth) + vy * std::sin(azimuth);
            detection.range_rate_std = 0.1f;
            detection.azimuth = azimuth;
            detection.range = 10.0f;
            detection.x = detection.range * std::cos(detection.azimuth);
            detection.y = detection.range * std::sin(detection.azimuth);
            detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
            return detection;
        }
    );

    calibration_.neighbourhood_threshold = 1000000.0f;
    calibration_.minimum_detection_in_segment = 2u;

    auto segmentator = measurements::radar::Segmentator(calibration_);

    segmentator.Run(scan);

    for (const auto & detection : scan.detections)
    {
        EXPECT_EQ(detection.segment_id, 1u);
    }
}

TEST_F(SegmentatorTests, RunTwoSegmentsTest) {
    measurements::radar::RadarScan scan;
    scan.detections.resize(20);
    auto vx = 10.0f;
    auto vy = 0.5f;
    std::vector<float> azimuths(scan.detections.size());
    std::iota(azimuths.begin(), azimuths.end(), -5);
    std::transform(azimuths.begin(), azimuths.end(), azimuths.begin(), [](float azimuth) { return azimuth * std::numbers::pi / 180.0f; });

    float range = 10.0f;
    size_t first_section_border = 10u;

    auto set_detection = [&](float azimuth) {
        static size_t id = 1;
        measurements::radar::RadarDetection detection;
        detection.id = id++;
        detection.range_rate = vx * std::cos(azimuth) + vy * std::sin(azimuth);
        detection.range_rate_std = 0.1f;
        detection.azimuth = azimuth;
        detection.range = range;
        detection.x = detection.range * std::cos(detection.azimuth);
        detection.y = detection.range * std::sin(detection.azimuth);
        detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
        return detection;
    };
    
    std::transform(azimuths.begin(), azimuths.begin() + first_section_border, scan.detections.begin(), set_detection);

    range = 100.0f;
    std::transform(azimuths.begin() + first_section_border, azimuths.end(), scan.detections.begin() + first_section_border, set_detection);

    calibration_.neighbourhood_threshold = 10.0f;
    calibration_.minimum_detection_in_segment = 2u;

    auto segmentator = measurements::radar::Segmentator(calibration_);

    segmentator.Run(scan);

    for (const auto & detection : scan.detections)
    {
        if (detection.id <= first_section_border)
            EXPECT_EQ(detection.segment_id, 1u);
        else
            EXPECT_EQ(detection.segment_id, 2u);
    }
}
