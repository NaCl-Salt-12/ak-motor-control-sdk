#pragma once

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <span>

#include "motor_control_sdk/lib/motor_specs.h"


namespace protocol::force_control {
    // The Control Mode ID for all Force Control commands
    constexpr uint32_t FORCE_CONTROL_ID = 8;

    namespace internal {
        inline int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
            float span = x_max - x_min;
            x = std::clamp(x, x_min, x_max);
            return static_cast<int>((x - x_min) * (static_cast<float>((1 << bits) - 1) / span));
        }
    }

    struct ForceControlCommand {
        float position_setpoint;    // radians
        float velocity_setpoint;    // rad/s
        float feedforward_torque;   // N.m
        float kp;                   // position gain (stiffness)
        float kd;                   // velocity gain (damping)
    };

    /**
     * @brief Creates a CAN frame for the Force Control (MIT) mode.
     * @param id The motor's ID (0-255).
     * @param command The ForceControlCommand struct.
     * @param limits The MotorLimits struct for the specific motor model.
     * @return A ready-to-send can_frame.
     */
    inline can_frame create_force_control_frame(
        uint8_t id,
        const ForceControlCommand& command,
        const motor_specs::MotorLimits& limits
    ) {
        const int p_int = internal::float_to_uint(command.position_setpoint, limits.POSITION_MIN, limits.POSITION_MAX, 16);
        const int v_int = internal::float_to_uint(command.velocity_setpoint, limits.VELOCITY_MIN, limits.VELOCITY_MAX, 12);
        const int t_int = internal::float_to_uint(command.feedforward_torque, limits.TORQUE_MIN, limits.TORQUE_MAX, 12);
        const int kp_int = internal::float_to_uint(command.kp, limits.KP_MIN, limits.KP_MAX, 12);
        const int kd_int = internal::float_to_uint(command.kd, limits.KD_MIN, limits.KD_MAX, 12);

        return can_frame{
            .can_id = static_cast<canid_t>(id | (FORCE_CONTROL_ID << 8) | CAN_EFF_FLAG),
            .can_dlc = 8,
            .data = {
                static_cast<uint8_t>(kp_int >> 4),
                static_cast<uint8_t>(((kp_int & 0x0F) << 4) | (kd_int >> 8)),
                static_cast<uint8_t>(kd_int & 0xFF),
                static_cast<uint8_t>(p_int >> 8),
                static_cast<uint8_t>(p_int & 0xFF),
                static_cast<uint8_t>(v_int >> 4),
                static_cast<uint8_t>(((v_int & 0x0F) << 4) | (t_int >> 8)),
                static_cast<uint8_t>(t_int & 0xFF)
            }
        };
    }

} // namespace protocol::force_control