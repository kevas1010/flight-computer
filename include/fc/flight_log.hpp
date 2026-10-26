#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "fc/flight_computer.hpp"

namespace fc::flightlog {

// File layout (little-endian):
//   "FCLOG01\0" | u32 config_len | config bytes
//   repeated: u32 record_len | record
// record = u32 tick | u8 n_sensor | SensorSample[n] | u8 n_cmd | Command[n]
//          | ActuatorCmd | Telemetry | u8 n_ack | Ack[n]
//
// Every input the flight computer saw and every output it produced, per frame,
// so a flight can be re-run bit-for-bit through FlightComputer::step().

struct TickRecord {
    std::uint32_t tick = 0;
    std::vector<proto::SensorSample> sensors;
    std::vector<proto::Command> commands;
    proto::ActuatorCmd actuator;
    proto::Telemetry telemetry;
    std::vector<proto::Ack> acks;
};

class Writer {
public:
    Writer() = default;
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    ~Writer();

    bool open(const std::string& path, const FcConfig& cfg);
    bool is_open() const { return file_ != nullptr; }
    void write(std::uint32_t tick, const FcInputs& in, const FcOutputs& out);
    void close();

private:
    std::FILE* file_ = nullptr;
};

class Reader {
public:
    Reader() = default;
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    ~Reader();

    bool open(const std::string& path, FcConfig& cfg);
    // Returns false at end of file or on a malformed record (see error()).
    bool next(TickRecord& rec);
    const std::string& error() const { return error_; }

private:
    std::FILE* file_ = nullptr;
    std::vector<std::uint8_t> buf_;
    std::string error_;
};

}  // namespace fc::flightlog
