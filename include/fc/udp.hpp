#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace fc::net {

sockaddr_in make_addr(const std::string& host, std::uint16_t port);
// Parses "host:port".
sockaddr_in parse_endpoint(const std::string& endpoint);

// Non-blocking IPv4 UDP socket (RAII).
class UdpSocket {
public:
    UdpSocket();  // unbound; the kernel picks an ephemeral port on first send
    static UdpSocket bound(const std::string& host, std::uint16_t port);

    UdpSocket(UdpSocket&& other) noexcept;
    UdpSocket& operator=(UdpSocket&& other) noexcept;
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    ~UdpSocket();

    // Returns the datagram length, or nullopt if nothing is queued.
    std::optional<std::size_t> recv(std::span<std::uint8_t> buf, sockaddr_in* from = nullptr);
    bool send_to(std::span<const std::uint8_t> data, const sockaddr_in& to);

private:
    int fd_ = -1;
};

}  // namespace fc::net
