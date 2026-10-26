// Log-replay regression tests. Golden logs in tests/data were recorded with
// fc_closedloop; replaying them through the current code must reproduce every
// recorded frame. If a behaviour change is intended, regenerate them with
// scripts/regen_golden.sh and review the diff in the transitions it prints.

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "fc/replay.hpp"
#include "sim/closed_loop.hpp"

namespace {

std::string temp_log(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("fc_test_" + name + ".fclog")).string();
}

TEST(Replay, FreshRecordingReplaysExactly) {
    const std::string path = temp_log("fresh");
    const auto flown = sim::run_closed_loop(*sim::named_scenario("nominal"), {}, path);
    const auto r = fc::replay::run(path, 0.0);  // zero tolerance: same binary must be bit-exact
    EXPECT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.mismatched_ticks, 0u);
    EXPECT_EQ(r.final_state, flown.final_state);
    EXPECT_EQ(r.transitions.size(), flown.transitions.size());
    std::remove(path.c_str());
}

TEST(Replay, DetectsBehaviourChange) {
    const std::string path = temp_log("changed");
    sim::run_closed_loop(*sim::named_scenario("nominal"), {}, path);
    fc::FcConfig changed;
    changed.guidance.kp = 0.5;  // a "code change" to the landing controller
    const auto r = fc::replay::run(path, 1e-4, &changed);
    EXPECT_FALSE(r.ok);
    ASSERT_TRUE(r.first_mismatch.has_value());
    EXPECT_GT(r.mismatched_ticks, 0u);
    std::remove(path.c_str());
}

TEST(Replay, RejectsMissingAndCorruptFiles) {
    EXPECT_FALSE(fc::replay::run("/nonexistent/flight.fclog").ok);
    const std::string path = temp_log("corrupt");
    if (std::FILE* f = std::fopen(path.c_str(), "wb")) {
        std::fputs("not a flight log", f);
        std::fclose(f);
    }
    const auto r = fc::replay::run(path);
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
    std::remove(path.c_str());
}

TEST(Replay, TruncatedLogReportsError) {
    const std::string path = temp_log("truncated");
    sim::run_closed_loop(*sim::named_scenario("nominal"), {}, path);
    const auto size = std::filesystem::file_size(path);
    std::filesystem::resize_file(path, size - 7);
    const auto r = fc::replay::run(path);
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.error.find("truncated"), std::string::npos) << r.error;
    EXPECT_GT(r.ticks, 1000u);
    std::remove(path.c_str());
}

struct Golden {
    const char* file;
    fc::FlightState final_state;
    std::uint16_t must_see_faults;
};

class GoldenReplay : public ::testing::TestWithParam<Golden> {};

TEST_P(GoldenReplay, ReproducesRecordedFlight) {
    const Golden g = GetParam();
    const std::string path = std::string(FC_TEST_DATA_DIR) + "/" + g.file;
    ASSERT_TRUE(std::filesystem::exists(path)) << path << " missing; run scripts/regen_golden.sh";
    const auto r = fc::replay::run(path);
    EXPECT_TRUE(r.error.empty()) << r.error;
    if (r.first_mismatch) {
        const auto& m = *r.first_mismatch;
        ADD_FAILURE() << "diverged at tick " << m.tick << " on " << m.field << ": expected " << m.expected << " got "
                      << m.actual << " (" << r.mismatched_ticks << " frames differ)";
    }
    EXPECT_TRUE(r.ok);
    EXPECT_EQ(r.final_state, g.final_state);
    EXPECT_EQ(r.faults_seen & g.must_see_faults, g.must_see_faults);
}

INSTANTIATE_TEST_SUITE_P(
    Logs, GoldenReplay,
    ::testing::Values(Golden{"nominal.fclog", fc::FlightState::Landed, 0},
                      Golden{"alt_dropout.fclog", fc::FlightState::Landed, fc::kFaultAltStale},
                      Golden{"imu_stuck.fclog", fc::FlightState::Landed, fc::kFaultImuStuck},
                      Golden{"dual_fault.fclog", fc::FlightState::Abort, fc::kFaultImuStuck | fc::kFaultAltStale}),
    [](const auto& p) {
        std::string n = p.param.file;
        return n.substr(0, n.find('.'));
    });

}  // namespace
