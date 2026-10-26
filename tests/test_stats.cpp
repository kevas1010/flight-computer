#include <gtest/gtest.h>

#include <vector>

#include "fc/stats.hpp"

namespace {

TEST(Stats, NearestRankPercentiles) {
    std::vector<double> v;
    for (int i = 1; i <= 100; ++i) v.push_back(i);
    const auto s = fc::summarize(v);
    EXPECT_EQ(s.count, 100u);
    EXPECT_DOUBLE_EQ(s.p50, 50.0);
    EXPECT_DOUBLE_EQ(s.p90, 90.0);
    EXPECT_DOUBLE_EQ(s.p99, 99.0);
    EXPECT_DOUBLE_EQ(s.max, 100.0);
    EXPECT_DOUBLE_EQ(s.mean, 50.5);
}

TEST(Stats, UnsortedInputAndSingleSample) {
    const std::vector<double> v{5.0, 1.0, 3.0};
    EXPECT_DOUBLE_EQ(fc::summarize(v).p50, 3.0);
    const std::vector<double> one{7.0};
    const auto s = fc::summarize(one);
    EXPECT_DOUBLE_EQ(s.p99, 7.0);
    EXPECT_DOUBLE_EQ(s.p50, 7.0);
    EXPECT_EQ(fc::summarize({}).count, 0u);
}

}  // namespace
