#include "gtest/gtest.h"

#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>

#include "segmentator_calibration.hpp"
#include "segmentator.hpp"
#include "distance_matrix.hpp"

/****************************** Distance Matrix *******************************/
class DistanceMatrixTests : public ::testing::Test
{
    protected:
        void SetUp(void) override {
        }

        void TearDown(void) override {
        }
};

TEST_F(DistanceMatrixTests, ConstructorTest) {
    EXPECT_NO_THROW(auto distances = std::unique_ptr<measurements::radar::DistanceMatrix>());
}

TEST_F(DistanceMatrixTests, SetSizeAndGetDataCorrectIndicesTest) {
    auto distances = measurements::radar::DistanceMatrix();

    constexpr size_t size = 5u;
    distances.SetSize(size);

    for (auto r = 0u; r < size; r++) {
        for (auto c = 0u; c <= r; c++) {
            EXPECT_FLOAT_EQ(distances(r, c), 0.0f);
            EXPECT_FLOAT_EQ(distances(r, c), distances(c, r));
        }
    }
}

TEST_F(DistanceMatrixTests, SetSizeAndGetDataIncorrectIndicesTest) {
    auto distances = measurements::radar::DistanceMatrix();

    constexpr size_t size = 5u;
    distances.SetSize(size);

    EXPECT_THROW(distances(0, 5), std::invalid_argument);
}

/******************************** Segmentator ********************************/
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

TEST_F(SegmentatorTests, RunEmptyScanTest) {
    measurements::radar::RadarScan scan;
    
    calibration_.probability_hreshold = 0.01f;
    calibration_.minimum_detection_in_segment = 2u;

    auto segmentator = measurements::radar::Segmentator(calibration_);

    EXPECT_NO_THROW(segmentator.Run(scan));
}

TEST_F(SegmentatorTests, RunAllDetectionInSingleSegmentTest) {
    measurements::radar::RadarScan scan;
    scan.detections.resize(10);
    std::vector<float> azimuths(scan.detections.size());
    std::iota(azimuths.begin(), azimuths.end(), -5);
    
    std::transform(azimuths.begin(), azimuths.end(),
        scan.detections.begin(),
        [=](float azimuth) {
            static size_t id = 1;
            measurements::radar::RadarDetection detection;
            detection.id = id++;
            detection.range_rate = 0.0f;
            detection.range_rate_std = 0.5f;
            detection.azimuth = azimuth;
            detection.azimuth_std = 1.0f;
            detection.range = 10.0f;
            detection.range_std = 1.0f;
            detection.x = detection.range * std::cos(detection.azimuth);
            detection.x_std = std::sqrt( std::pow(std::cos(detection.azimuth) * detection.azimuth_std, 2.0f)
                + std::pow(detection.range * std::sin(detection.azimuth) * detection.azimuth_std, 2.0f) );
            detection.y = detection.range * std::sin(detection.azimuth);
            detection.y_std = std::sqrt( std::pow(std::sin(detection.azimuth) * detection.azimuth_std, 2.0f)
                + std::pow(detection.range * std::cos(detection.azimuth) * detection.azimuth_std, 2.0f) );
            detection.dealiasing_status = measurements::radar::DealiasingStatus::StaticVelocityProfileDealiased;
            return detection;
        }
    );

    calibration_.probability_hreshold = 0.9999f;
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

    calibration_.probability_hreshold = 0.9f;
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

TEST_F(SegmentatorTests, RunSegmentedPointsTest) {
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
            detection.segment_id = 1u;
            return detection;
        }
    );

    calibration_.probability_hreshold = 0.001f;
    calibration_.minimum_detection_in_segment = 2u;

    auto segmentator = measurements::radar::Segmentator(calibration_);

    segmentator.Run(scan);

    for (const auto & detection : scan.detections)
    {
        EXPECT_EQ(detection.segment_id, 1u);
    }
}
