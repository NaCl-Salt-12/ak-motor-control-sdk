#pragma once

#include <string_view>
#include <expected>
#include <system_error>
#include <mutex>
#include <chrono>

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
         * @brief Sets a timeout for read operations using a type-safe duration.
         * @param timeout The desired timeout duration. A zero duration can be used for non-blocking.
         * @return A std::expected that is empty on success or contains an error.
         */
        auto set_read_timeout(std::chrono::milliseconds timeout) -> expected_void<std::errc>;

        /**
         * @brief Writes a CAN frame to the bus (thread-safe).
         * @param frame The can_frame to send.
         * @return A std::expected that is empty on success or contains an error.
         */
        auto write_frame(const can_frame& frame) const -> expected_void<std::errc>;

        /**
         * @brief Reads a CAN frame from the bus (thread-safe, blocking call).
         * @return A std::expected containing the can_frame on success or an error.
         * Returns std::errc::timed_out if a timeout was set and occurred.
         */
        auto read_frame() const -> std::expected<can_frame, std::errc>;

    private:
        int socket_fd_ = -1;
        mutable std::mutex socket_mutex_;
};
