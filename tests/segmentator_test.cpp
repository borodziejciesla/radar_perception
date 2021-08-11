#include "gtest/gtest.h"

#include <memory>

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
