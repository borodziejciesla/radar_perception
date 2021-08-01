#include "gtest/gtest.h"

#include "dealiaser.hpp"

class DealiaserTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}
};

TEST_F(DealiaserTests, ConstructorTest)
{
    std::unique_ptr<measurements::radar::Dealiaser> dealiaser;
    EXPECT_NO_THROW(dealiaser = std::make_unique<measurements::radar::Dealiaser>());
}

TEST_F(DealiaserTests, RunTest)
{
    measurements::radar::RadarScan scan;
    scan.detections.resize(10);
    
    measurements::radar::Dealiaser dealiaser;
    dealiaser.Run(scan);

    EXPECT_TRUE(true);
}
