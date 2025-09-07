#include "can_bus.h"

#include <stdexcept>
#include <cstring>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>


CanBus::CanBus(std::string_view interface) {
    // 1. Create the socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        throw std::system_error(
            errno,
            std::generic_category(), 
            "Failed to create CAN socket"
        );
    }

    // 2. Get the interface index from its name (e.g., "can0")
    ifreq ifr{};
    strncpy(ifr.ifr_name, interface.data(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // Ensure null termination

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd_); // Clean up on failure
        throw std::system_error(
            errno,
            std::generic_category(), 
            "Failed to get interface index for " + std::string(interface)
        );
    }

    // 3. Bind the socket to the CAN interface
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
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

auto CanBus::write_frame(const can_frame& frame) const -> expected_void<std::errc> {
    if (write(socket_fd_, &frame, sizeof(can_frame)) != sizeof(can_frame)) {
        return std::unexpected(static_cast<std::errc>(errno));
    }

    return {};
}

auto CanBus::read_frame() const -> std::expected<can_frame, std::errc> {
    can_frame frame{};
    ssize_t bytes_read = read(socket_fd_, &frame, sizeof(can_frame));

    if (bytes_read < 0) {
        return std::unexpected(static_cast<std::errc>(errno));
    }
    
    if (bytes_read < sizeof(can_frame)) {
        // This indicates a malformed or partial frame read
        return std::unexpected(std::errc::io_error);
    }

    return frame;
}