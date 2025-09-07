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
    inline can_frame create_force_control_frame(
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

} // namespace protocol::force_control