#include "gtest/gtest.h"

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
    
    measurements::radar::Dealiaser dealiaser(calibrations_);
    dealiaser.Run(scan);

    EXPECT_TRUE(true);
}
