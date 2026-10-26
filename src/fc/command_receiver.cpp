#include "fc/command_receiver.hpp"

namespace fc {

std::optional<proto::Ack> CommandReceiver::duplicate_of(const proto::Command& cmd) {
    if (!have_session_ || cmd.session != session_) {
        // A new ground-station session starts a fresh id space.
        have_session_ = true;
        session_ = cmd.session;
        last_id_ = 0;
        history_ = {};
        return std::nullopt;
    }
    if (cmd.cmd_id > last_id_) return std::nullopt;

    ++duplicates_;
    const proto::Ack& cached = history_[cmd.cmd_id % kHistory];
    if (cached.cmd_id == cmd.cmd_id && cached.session == cmd.session) return cached;
    // Too old to still be in the history: it was executed, but the result is gone.
    return proto::Ack{cmd.session, cmd.cmd_id, cmd.opcode, proto::AckResult::Rejected};
}

void CommandReceiver::record(const proto::Ack& ack) {
    ++accepted_;
    last_id_ = ack.cmd_id;
    history_[ack.cmd_id % kHistory] = ack;
}

}  // namespace fc
