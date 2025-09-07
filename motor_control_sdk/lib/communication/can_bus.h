#pragma once

#include <string_view>
#include <expected>
#include <system_error>

#include <linux/can.h>


template<class E>
using expected_void = std::expected<void, E>;

class CanBus {
public:
    /**
     * @brief Constructs a CanBus object and opens the socket.
     * @param interface The network interface name (e.g., "can0").
     * @throws std::system_error if the socket cannot be created, configured, or bound.
     */
    explicit CanBus(std::string_view interface);

    /**
     * @brief Destructor automatically closes the socket.
     */
    ~CanBus();

    CanBus(const CanBus&) = delete;
    CanBus& operator=(const CanBus&) = delete;
    CanBus(CanBus&&) = delete;
    CanBus& operator=(CanBus&&) = delete;

    /**
     * @brief Writes a CAN frame to the bus.
     * @param frame The can_frame to send.
     * @return A std::expected that is empty on success or contains an error.
     */
    auto write_frame(const can_frame& frame) const -> expected_void<std::errc>;

    /**
     * @brief Reads a CAN frame from the bus (this is a blocking call).
     * @return A std::expected containing the can_frame on success or an error.
     */
    auto read_frame() const -> std::expected<can_frame, std::errc>;

private:
    int socket_fd_ = -1;
};