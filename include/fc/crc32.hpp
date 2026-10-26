#pragma once

#include <cstdint>
#include <span>

namespace fc {

// CRC-32/ISO-HDLC (IEEE 802.3, the zlib/PNG CRC): reflected polynomial
// 0xEDB88320, init and final XOR 0xFFFFFFFF. Matches Python's zlib.crc32.
std::uint32_t crc32(std::span<const std::uint8_t> data);

}  // namespace fc
