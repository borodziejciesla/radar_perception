#include "gtest/gtest.h"

#include <numeric>
#include <cmath>
#include <numbers>

#include "detection_classifier.hpp"

class DetectionClassifierTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}
};

TEST_F(DetectionClassifierTests, ConstructorTest)
{
    std::unique_ptr<measurements::radar::DetectionClassifier> dealiaser_classifier;
    EXPECT_NO_THROW(dealiaser_classifier = std::make_unique<measurements::radar::DetectionClassifier>());
}

TEST_F(DetectionClassifierTests, RunAllStaticTest)
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

    measurements::radar::VelocityProfile vp;
    vp.value.at(0u) = vx;
    vp.value.at(1u) = vy;
    vp.covariance.covariance_diagonal.at(0u) = 0.1f;
    vp.covariance.covariance_diagonal.at(1u) = 0.1f;
    vp.covariance.covariance_lower_triangle.at(0u) = 0.0f;

    auto dealiaser_classifier = measurements::radar::DetectionClassifier();
    dealiaser_classifier.Run(scan, vp);

    for (const auto & detection : scan.detections)
        EXPECT_EQ(detection.moving_status, measurements::radar::MovingStatus::Static);
}

TEST_F(DetectionClassifierTests, RunAllMovingTest)
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
            static size_t id = 1;
            measurements::radar::RadarDetection detection;
            detection.id = id++;
            detection.range_rate = vx * std::cos(azimuth) + vy * std::sin(azimuth) + 5.0f;
            detection.range_rate_std = 0.1f;
            detection.azimuth = azimuth;
            detection.range = 10.0f;
            detection.x = detection.range * std::cos(detection.azimuth);
            detection.y = detection.range * std::sin(detection.azimuth);
            detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
            return detection;
        }
    );

    measurements::radar::VelocityProfile vp;
    vp.value.at(0u) = vx;
    vp.value.at(1u) = vy;
    vp.covariance.covariance_diagonal.at(0u) = 0.1f;
    vp.covariance.covariance_diagonal.at(1u) = 0.1f;
    vp.covariance.covariance_lower_triangle.at(0u) = 0.0f;

    auto dealiaser_classifier = measurements::radar::DetectionClassifier();
    dealiaser_classifier.Run(scan, vp);

    for (const auto & detection : scan.detections)
        EXPECT_EQ(detection.moving_status, measurements::radar::MovingStatus::Ambiguous);
}
