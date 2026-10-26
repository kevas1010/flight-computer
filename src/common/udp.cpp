#include "fc/udp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace fc::net {
namespace {

[[noreturn]] void throw_errno(const char* what) { throw std::system_error(errno, std::generic_category(), what); }

}  // namespace

sockaddr_in make_addr(const std::string& host, std::uint16_t port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) throw std::invalid_argument("bad IPv4 address: " + host);
    return addr;
}

sockaddr_in parse_endpoint(const std::string& endpoint) {
    const auto colon = endpoint.rfind(':');
    if (colon == std::string::npos) throw std::invalid_argument("expected host:port, got " + endpoint);
    const int port = std::stoi(endpoint.substr(colon + 1));
    if (port <= 0 || port > 65535) throw std::invalid_argument("bad port in " + endpoint);
    return make_addr(endpoint.substr(0, colon), static_cast<std::uint16_t>(port));
}

UdpSocket::UdpSocket() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) throw_errno("socket");
}

UdpSocket UdpSocket::bound(const std::string& host, std::uint16_t port) {
    UdpSocket s;
    const int one = 1;
    ::setsockopt(s.fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    const sockaddr_in addr = make_addr(host, port);
    if (::bind(s.fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) < 0) throw_errno("bind");
    return s;
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
}

UdpSocket::~UdpSocket() {
    if (fd_ >= 0) ::close(fd_);
}

std::optional<std::size_t> UdpSocket::recv(std::span<std::uint8_t> buf, sockaddr_in* from) {
    sockaddr_in src{};
    socklen_t len = sizeof src;
    const ssize_t n = ::recvfrom(fd_, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&src), &len);
    if (n < 0) return std::nullopt;  // EAGAIN, or a transient error such as a queued ICMP refusal
    if (from != nullptr) *from = src;
    return static_cast<std::size_t>(n);
}

bool UdpSocket::send_to(std::span<const std::uint8_t> data, const sockaddr_in& to) {
    const ssize_t n =
        ::sendto(fd_, data.data(), data.size(), 0, reinterpret_cast<const sockaddr*>(&to), sizeof to);
    return n == static_cast<ssize_t>(data.size());
}

}  // namespace fc::net
