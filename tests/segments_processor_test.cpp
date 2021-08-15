#include "gtest/gtest.h"

#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>
#include <memory>
#include <numbers>

#include "segment_processor.hpp"
#include "covariance.hpp"
#include "segmentator.hpp"

/*************************** Covariance Test ***************************/
TEST(CovarianceTest, CreateTest) {
    measurements::radar::Covariance<2u> c2;
    EXPECT_EQ(c2.covariance_diagonal.size(), 2u);
    EXPECT_EQ(c2.covariance_lower_triangle.size(), 1u);

    for (const auto & element : c2.covariance_diagonal)
        EXPECT_FLOAT_EQ(element, 0.0f);
    for (const auto & element : c2.covariance_lower_triangle)
        EXPECT_FLOAT_EQ(element, 0.0f);

    measurements::radar::Covariance<3u> c3;
    EXPECT_EQ(c3.covariance_diagonal.size(), 3u);
    EXPECT_EQ(c3.covariance_lower_triangle.size(), 3u);

    for (const auto & element : c3.covariance_diagonal)
        EXPECT_FLOAT_EQ(element, 0.0f);
    for (const auto & element : c3.covariance_lower_triangle)
        EXPECT_FLOAT_EQ(element, 0.0f);

    measurements::radar::Covariance<4u> c4;
    EXPECT_EQ(c4.covariance_diagonal.size(), 4u);
    EXPECT_EQ(c4.covariance_lower_triangle.size(), 6u);

    for (const auto & element : c4.covariance_diagonal)
        EXPECT_FLOAT_EQ(element, 0.0f);
    for (const auto & element : c4.covariance_lower_triangle)
        EXPECT_FLOAT_EQ(element, 0.0f);

    measurements::radar::Covariance<5u> c5;
    EXPECT_EQ(c5.covariance_diagonal.size(), 5u);
    EXPECT_EQ(c5.covariance_lower_triangle.size(), 10u);

    for (const auto & element : c5.covariance_diagonal)
        EXPECT_FLOAT_EQ(element, 0.0f);
    for (const auto & element : c5.covariance_lower_triangle)
        EXPECT_FLOAT_EQ(element, 0.0f);
}

/*********************** Segments Processor Test ***********************/
class SegmentsProcessorTests : public ::testing::Test
{
    protected:
        void SetUp(void) override {
        }

        void TearDown(void) override {
        }

        measurements::radar::SegmentProcessorCalibration calibration_;
};

TEST_F(SegmentsProcessorTests, ConstructorTest) {
    EXPECT_NO_THROW(auto processor = std::make_unique<measurements::radar::SegmentsProcessor>(calibration_));
}

TEST_F(SegmentsProcessorTests, RunEmptyScanTest) {
    measurements::radar::RadarScan scan;
    measurements::radar::VelocityProfile vp;
    measurements::radar::SegmentsProcessor processor(calibration_);

    auto [objects, guardrials] = processor.ProcessSegments(scan, vp);
}

TEST_F(SegmentsProcessorTests, RunStaticRadarOneStaticSegmentTest) {
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
        scan.detections.push_back(detection);
    }

    measurements::radar::VelocityProfile vp;
    vp.vx = 0.0f;
    vp.vy = 0.0f;

    measurements::radar::SegmentatorCalibration segmentator_calibration;
    segmentator_calibration.neighbourhood_threshold = 1.0f;
    segmentator_calibration.minimum_detection_in_segment = 2u;

    auto segmentator = measurements::radar::Segmentator(segmentator_calibration);
    segmentator.Run(scan);

    measurements::radar::SegmentsProcessor processor(calibration_);
    auto [objects, guardrials] = processor.ProcessSegments(scan, vp);

    // Objects
    EXPECT_TRUE(objects.empty());

    // Guardrails
    EXPECT_EQ(guardrials.size(), 1u);
    EXPECT_FLOAT_EQ(guardrials.at(0).range.start, 5.0f);
    EXPECT_FLOAT_EQ(guardrials.at(0).range.end, 9.5f);
}

TEST_F(SegmentsProcessorTests, RunStaticRadarTwoStaticSegmentTest) {
    measurements::radar::RadarScan scan;
    // first segment
    auto det_id = 0u;
    for (auto index = 0u; index < 10u; index++) {
        measurements::radar::RadarDetection detection;
        detection.id = ++det_id;
        detection.x = 5.0f + static_cast<float>(index) * 0.5f;
        detection.x_std = 0.25f;
        detection.y = 5.0f;
        detection.y_std = 0.25f;
        detection.range = std::hypot(detection.x, detection.y);
        detection.azimuth = std::atan2(detection.y, detection.x);
        scan.detections.push_back(detection);
    }
    //second segment
    for (auto index = 0u; index < 10u; index++) {
        measurements::radar::RadarDetection detection;
        detection.id = ++det_id;
        detection.x = static_cast<float>(index) * 0.5f;
        detection.x_std = 0.25f;
        detection.y = -5.0f;
        detection.y_std = 0.25f;
        detection.range = std::hypot(detection.x, detection.y);
        detection.azimuth = std::atan2(detection.y, detection.x);
        scan.detections.push_back(detection);
    }

    measurements::radar::VelocityProfile vp;
    vp.vx = 0.0f;
    vp.vy = 0.0f;

    measurements::radar::SegmentatorCalibration segmentator_calibration;
    segmentator_calibration.neighbourhood_threshold = 1.0f;
    segmentator_calibration.minimum_detection_in_segment = 2u;

    auto segmentator = measurements::radar::Segmentator(segmentator_calibration);
    segmentator.Run(scan);

    measurements::radar::SegmentsProcessor processor(calibration_);
    auto [objects, guardrials] = processor.ProcessSegments(scan, vp);

    // Objects
    EXPECT_TRUE(objects.empty());

    // Guardrails
    EXPECT_EQ(guardrials.size(), 2u);

    EXPECT_FLOAT_EQ(guardrials.at(0).range.start, 5.0f);
    EXPECT_FLOAT_EQ(guardrials.at(0).range.end, 9.5f);

    EXPECT_FLOAT_EQ(guardrials.at(1).range.start, 0.0f);
    EXPECT_FLOAT_EQ(guardrials.at(1).range.end, 4.5f);
}