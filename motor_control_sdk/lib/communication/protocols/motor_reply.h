#pragma once

#include <cstdint>
#include <span>


namespace protocol {
    /**
     * @brief Represents a reply message received from the motor.
     * @param position Position in degrees.
     * @param velocity Velocity in electrical RPM.
     * @param current Current in Amps.
     * @param temperature Temperature in Celsius.
     * @param error_code Error code as per the motor's documentation.
     */
    struct MotorReply {
        float position;
        float velocity;
        float current;
        int8_t temperature;
        uint8_t error_code;
    };

    /**
     * @brief Unpacks an 8-byte CAN reply frame into a MotorReply struct.
     * @param data An 8-byte span from the `can_frame.data` field.
     * @return A struct containing the parsed motor state.
     */
    inline MotorReply unpack_reply_frame(std::span<const uint8_t, 8> data) {
        // Scaling factors from the motor documentation
        constexpr float POSITION_SCALE = 0.1f;
        constexpr float VELOCITY_SCALE = 10.0f;
        constexpr float CURRENT_SCALE = 0.01f;

        const int16_t pos_int = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        const int16_t vel_int = (static_cast<uint16_t>(data[2]) << 8) | data[3];
        const int16_t cur_int = (static_cast<uint16_t>(data[4]) << 8) | data[5];

        return {
            .position = static_cast<float>(pos_int) * POSITION_SCALE,
            .velocity = static_cast<float>(vel_int) * VELOCITY_SCALE,
            .current = static_cast<float>(cur_int) * CURRENT_SCALE,
            .temperature = static_cast<int8_t>(data[6]),
            .error_code = data[7]
        };
    }

} // namespace protocol