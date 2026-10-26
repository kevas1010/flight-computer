#pragma once

#include <cstddef>
#include <span>

namespace fc {

struct LatencySummary {
    std::size_t count = 0;
    double mean = 0.0;
    double p50 = 0.0;
    double p90 = 0.0;
    double p99 = 0.0;
    double p999 = 0.0;
    double max = 0.0;
};

// Nearest-rank percentile of an ascending-sorted range. `p` is in [0, 100].
double percentile_sorted(std::span<const double> sorted, double p);

// Summary statistics over `samples` (copied and sorted internally).
LatencySummary summarize(std::span<const double> samples);

}  // namespace fc
