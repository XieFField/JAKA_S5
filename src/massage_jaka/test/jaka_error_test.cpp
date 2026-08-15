#include <gtest/gtest.h>

#include "massage_jaka/jaka_error.hpp"

TEST(JakaErrorTest, MapsKnownSdkErrors)
{
    EXPECT_EQ(
        massage_jaka::map_jaka_error(-3),
        massage_jaka::JakaError::kCommunication);
    EXPECT_EQ(
        massage_jaka::map_jaka_error(-5),
        massage_jaka::JakaError::kEmergencyStop);
    EXPECT_EQ(
        massage_jaka::map_jaka_error(-7),
        massage_jaka::JakaError::kNotEnabled);
}

TEST(JakaErrorTest, PreservesUnknownErrors)
{
    EXPECT_EQ(
        massage_jaka::map_jaka_error(-999),
        massage_jaka::JakaError::kUnknown);
    EXPECT_EQ(
        massage_jaka::to_string(massage_jaka::JakaError::kUnknown),
        "unknown");
}
