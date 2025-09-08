#include "can_bus.h"

#include <string_view>
#include <expected>
#include <system_error>
#include <mutex>
#include <chrono>
#include <stdexcept>
#include <cstring>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>


CanBus::CanBus(std::string_view interface) {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw std::system_error(
            errno,
            std::generic_category(), 
            "Failed to create CAN socket"
        );
    }

    ifreq ifr{};
    strncpy(ifr.ifr_name, interface.data(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_);
        throw std::system_error(
            errno,
            std::generic_category(), 
            "Failed to get interface index for " + std::string(interface)
        );
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(socket_fd_);
        throw std::system_error(
            errno, 
            std::generic_category(), 
            "Failed to bind socket to " + std::string(interface)
        );
    }
}

CanBus::~CanBus() {
    if (socket_fd_ >= 0)
        close(socket_fd_);
}

auto CanBus::set_read_timeout(std::chrono::milliseconds timeout) -> expected_void<std::errc> {
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(timeout);
    auto remainder = timeout - secs;
    auto usecs = std::chrono::duration_cast<std::chrono::microseconds>(remainder);

    timeval tv{};
    tv.tv_sec = secs.count();
    tv.tv_usec = usecs.count();

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        return std::unexpected(static_cast<std::errc>(errno));

    return {};
}

auto CanBus::write_frame(const can_frame& frame) const -> expected_void<std::errc> {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (write(socket_fd_, &frame, sizeof(can_frame)) != sizeof(can_frame))
        return std::unexpected(static_cast<std::errc>(errno));

    return {};
}

auto CanBus::read_frame() const -> std::expected<can_frame, std::errc> {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    can_frame frame{};
    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(can_frame));

    if (bytes_read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return std::unexpected(std::errc::timed_out);
        }
        return std::unexpected(static_cast<std::errc>(errno));
    }
    
    if (bytes_read < sizeof(can_frame))
        return std::unexpected(std::errc::io_error);

    return frame;
}
