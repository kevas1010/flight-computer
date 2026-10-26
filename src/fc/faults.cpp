#include "fc/faults.hpp"

#include <bit>

namespace fc {

bool ImuStuckDetector::update(float accel) {
    const bool same = have_last_ && std::bit_cast<std::uint32_t>(accel) == std::bit_cast<std::uint32_t>(last_);
    have_last_ = true;
    last_ = accel;
    if (same) {
        ++repeats_;
        changes_ = 0;
        if (repeats_ + 1 >= stuck_samples_) stuck_ = true;
    } else {
        repeats_ = 0;
        ++changes_;
        if (stuck_ && changes_ >= clear_samples_) stuck_ = false;
    }
    return stuck_;
}

}  // namespace fc
