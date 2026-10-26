#include "fc/stats.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace fc {

double percentile_sorted(std::span<const double> sorted, double p) {
    if (sorted.empty()) return 0.0;
    const double rank = std::ceil(std::clamp(p, 0.0, 100.0) / 100.0 * static_cast<double>(sorted.size()));
    const auto idx = static_cast<std::size_t>(std::max(rank, 1.0)) - 1;
    return sorted[std::min(idx, sorted.size() - 1)];
}

LatencySummary summarize(std::span<const double> samples) {
    LatencySummary s;
    if (samples.empty()) return s;
    std::vector<double> sorted(samples.begin(), samples.end());
    std::sort(sorted.begin(), sorted.end());
    s.count = sorted.size();
    s.mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(s.count);
    s.p50 = percentile_sorted(sorted, 50.0);
    s.p90 = percentile_sorted(sorted, 90.0);
    s.p99 = percentile_sorted(sorted, 99.0);
    s.p999 = percentile_sorted(sorted, 99.9);
    s.max = sorted.back();
    return s;
}

}  // namespace fc
