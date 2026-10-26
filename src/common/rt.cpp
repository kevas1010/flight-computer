#include "fc/rt.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <time.h>

#include <cerrno>
#include <cstring>

namespace fc::rt {
namespace {

constexpr std::int64_t kNsPerSec = 1'000'000'000;

timespec to_timespec(std::int64_t ns) {
    timespec ts{};
    ts.tv_sec = static_cast<time_t>(ns / kNsPerSec);
    ts.tv_nsec = static_cast<long>(ns % kNsPerSec);
    return ts;
}

}  // namespace

std::int64_t monotonic_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec;
}

PeriodicTimer::PeriodicTimer(std::int64_t period_ns, std::int64_t spin_ns)
    : period_ns_(period_ns), spin_ns_(spin_ns), next_ns_(monotonic_ns()) {}

std::int64_t PeriodicTimer::wait() {
    next_ns_ += period_ns_;
    const std::int64_t now = monotonic_ns();
    if (now >= next_ns_) {
        const std::int64_t late = now - next_ns_;
        if (late >= period_ns_) {
            ++overruns_;
            next_ns_ = now;
        }
        return late;
    }
    const timespec wake = to_timespec(next_ns_ - spin_ns_);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR) {
    }
    std::int64_t t = monotonic_ns();
    while (t < next_ns_) t = monotonic_ns();  // no-op when spin_ns_ == 0 and the sleep was exact
    return t - next_ns_;
}

void minimize_timer_slack() { prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL); }

bool enable_realtime(int priority, std::string& why) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        why = std::string("mlockall: ") + std::strerror(errno);
        return false;
    }
    sched_param sp{};
    sp.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        why = std::string("sched_setscheduler(SCHED_FIFO): ") + std::strerror(errno);
        return false;
    }
    return true;
}

}  // namespace fc::rt
