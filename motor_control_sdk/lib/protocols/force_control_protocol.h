#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <span>

#include "motor_control_sdk/lib/motor_specs.h"


namespace force_control_protocol {
    // The Control Mode ID for all Force Control commands
    constexpr uint32_t FORCE_CONTROL_ID = 8;

    namespace internal {
        inline int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
            float span = x_max - x_min;
            x = std::clamp(x, x_min, x_max);
            return static_cast<int>((x - x_min) * (static_cast<float>((1 << bits) - 1) / span));
        }
    }

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
        MotorReply reply;

        int16_t pos_int = (data[0] << 8) | data[1];
        int16_t vel_int = (data[2] << 8) | data[3];
        int16_t cur_int = (data[4] << 8) | data[5];

        reply.position = static_cast<float>(pos_int) * 0.1f;
        reply.velocity = static_cast<float>(vel_int) * 10.0f;
        reply.current = static_cast<float>(cur_int) * 0.01f;
        reply.temperature = data[6];
        reply.error_code = data[7];
        return reply;
    }

    /**
     * @brief Creates a CAN frame for the Force Control (MIT) mode.
     * @param driver_id The motor's ID (0-255).
     * @param position Desired position in radians.
     * @param velocity Desired velocity in rad/s.
     * @param feedforward_torque Feed-forward torque in N.m.
     * @param kp Position gain (stiffness).
     * @param kd Velocity gain (damping).
     * @param limits The MotorLimits struct for the specific motor model.
     * @return A ready-to-send can_frame.
     */
    inline can_frame create_mit_frame(
        uint8_t driver_id,
        float position_setpoint,
        float velocity_setpoint,
        float feedforward_torque,
        float kp,
        float kd,
        const MotorLimits& limits
    ) {
        can_frame frame{};
        frame.can_id = driver_id | (FORCE_CONTROL_ID << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;

        int p_int = float_to_uint(position_setpoint, limits.POSITION_MIN, limits.POSITION_MAX, 16);
        int v_int = float_to_uint(velocity_setpoint, limits.VELOCITY_MIN, limits.VELOCITY_MAX, 12);
        int t_int = float_to_uint(feedforward_torque, limits.TORQUE_MIN, limits.TORQUE_MAX, 12);
        int kp_int = float_to_uint(kp, limits.KP_MIN, limits.KP_MAX, 12);
        int kd_int = float_to_uint(kd, limits.KD_MIN, limits.KD_MAX, 12);

        frame.data[0] = kp_int >> 4;
        frame.data[1] = ((kp_int & 0x0F) << 4) | (kd_int >> 8);
        frame.data[2] = kd_int & 0xFF;
        frame.data[3] = p_int >> 8;
        frame.data[4] = p_int & 0xFF;
        frame.data[5] = v_int >> 4;
        frame.data[6] = ((v_int & 0x0F) << 4) | (t_int >> 8);
        frame.data[7] = t_int & 0xFF;
        
        return frame;
    }

} // namespace mit_protocol