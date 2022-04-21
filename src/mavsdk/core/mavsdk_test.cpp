#include "mavsdk.h"
#include <gtest/gtest.h>

using namespace mavsdk;

TEST(Mavsdk, version)
{
    Mavsdk mavsdk;
    ASSERT_GT(mavsdk.version().size(), 5);
}

TEST(Mavsdk, configurationDescription)
{
    Mavsdk::Configuration configuration(Mavsdk::Configuration::UsageType::CompanionComputer);

    EXPECT_FALSE(configuration.get_component_description().has_value());
    configuration.set_component_description("test1234");
    EXPECT_TRUE(configuration.get_component_description().has_value());

    std::string test_reference("test1234");
    EXPECT_EQ(test_reference.compare(configuration.get_component_description().value()), 0);
}