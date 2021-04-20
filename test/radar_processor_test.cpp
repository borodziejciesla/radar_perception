#include "gtest/gtest.h"

#include "radar_processor.h"

class RadarProcessorTests : public ::testing::Test
{
    protected:
        void SetUp(void) override
        {}
};

TEST_F(RadarProcessorTests, ConstructorTest)
{
    measurements::radar::RadarProcessor rp;
    EXPECT_TRUE(true);
}
