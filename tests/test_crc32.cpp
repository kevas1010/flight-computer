#include <gtest/gtest.h>

#include <array>
#include <cstring>

#include "fc/crc32.hpp"

namespace {

std::span<const std::uint8_t> bytes(const char* s) {
    return {reinterpret_cast<const std::uint8_t*>(s), std::strlen(s)};
}

TEST(Crc32, MatchesStandardCheckValue) {
    // The catalogued check value for CRC-32/ISO-HDLC.
    EXPECT_EQ(fc::crc32(bytes("123456789")), 0xCBF43926u);
}

TEST(Crc32, EmptyInputIsZero) { EXPECT_EQ(fc::crc32({}), 0u); }

TEST(Crc32, MatchesZlibForKnownString) {
    // python3 -c "import zlib; print(hex(zlib.crc32(b'The quick brown fox jumps over the lazy dog')))"
    EXPECT_EQ(fc::crc32(bytes("The quick brown fox jumps over the lazy dog")), 0x414FA339u);
}

TEST(Crc32, DetectsEverySingleBitFlip) {
    std::array<std::uint8_t, 32> data{};
    for (std::size_t i = 0; i < data.size(); ++i) data[i] = static_cast<std::uint8_t>(i * 37 + 11);
    const std::uint32_t good = fc::crc32(data);
    for (std::size_t byte = 0; byte < data.size(); ++byte) {
        for (int bit = 0; bit < 8; ++bit) {
            data[byte] ^= static_cast<std::uint8_t>(1u << bit);
            EXPECT_NE(fc::crc32(data), good) << "byte " << byte << " bit " << bit;
            data[byte] ^= static_cast<std::uint8_t>(1u << bit);
        }
    }
}

}  // namespace
