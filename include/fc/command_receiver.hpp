#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "fc/protocol.hpp"

namespace fc {

// Receive side of the uplink protocol. The ground station retransmits a command
// until it sees the ACK (stop-and-wait), so the flight computer must execute each
// cmd_id at most once and answer retransmissions with the original ACK.
class CommandReceiver {
public:
    // If `cmd` was already executed, returns the ACK to resend; nullopt means
    // execute it, then call record().
    std::optional<proto::Ack> duplicate_of(const proto::Command& cmd);
    void record(const proto::Ack& ack);

    std::uint32_t accepted() const { return accepted_; }
    std::uint32_t duplicates() const { return duplicates_; }

private:
    static constexpr std::size_t kHistory = 16;

    bool have_session_ = false;
    std::uint32_t session_ = 0;
    std::uint32_t last_id_ = 0;
    std::array<proto::Ack, kHistory> history_{};
    std::uint32_t accepted_ = 0;
    std::uint32_t duplicates_ = 0;
};

}  // namespace fc
