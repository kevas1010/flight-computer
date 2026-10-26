#pragma once

#include <cstdint>
#include <string>

namespace fc::rt {

std::int64_t monotonic_ns();

// Fixed-rate scheduler on absolute CLOCK_MONOTONIC deadlines, so per-iteration
// work time does not accumulate into drift.
//
// With spin_ns > 0 it sleeps until spin_ns before the deadline and busy-waits the
// rest: trades one core's worth of CPU for wake-up latency below the kernel's
// timer resolution (useful on virtualised hosts such as WSL2).
class PeriodicTimer {
public:
    explicit PeriodicTimer(std::int64_t period_ns, std::int64_t spin_ns = 0);

    // Blocks until the next deadline and returns how late the wake-up was (ns).
    // If a whole period was missed, counts an overrun and re-anchors the schedule
    // instead of bursting through the missed slots.
    std::int64_t wait();

    std::int64_t period_ns() const { return period_ns_; }
    std::uint32_t overruns() const { return overruns_; }

private:
    std::int64_t period_ns_;
    std::int64_t spin_ns_;
    std::int64_t next_ns_;
    std::uint32_t overruns_ = 0;
};

// The kernel defers normal-priority timer wake-ups by up to 50 us by default to
// batch them; drop that to 1 ns for this thread.
void minimize_timer_slack();

// mlockall + SCHED_FIFO. Returns false (with a reason) if not permitted.
bool enable_realtime(int priority, std::string& why);

}  // namespace fc::rt
