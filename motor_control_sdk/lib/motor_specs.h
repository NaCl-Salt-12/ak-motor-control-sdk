#pragma once

#include <string>
#include <map>


namespace motor_specs {
    /**
     * @brief Holds the physical operating limits for a specific motor model.
     */
    struct MotorLimits {
        double POSITION_MIN, POSITION_MAX;      // Position [rad]
        double VELOCITY_MIN, VELOCITY_MAX;      // Velocity [rad/s]
        double TORQUE_MIN, TORQUE_MAX;          // Torque   [N.m]
        double KP_MIN, KP_MAX;                  // Stiffness
        double KD_MIN, KD_MAX;                  // Damping
    };

    /**
     * @brief A map of known motor limits, populated from the manual's table.
     * The key is the motor model string (e.g., "AK10-9").
     */
    const static std::map<std::string, MotorLimits> MOTOR_LIMITS = {
        {"AK10-9", {-12.56, 12.56, -28.0, 28.0, -54.0, 54.0, 0.0, 500.0, 0.0, 5.0}},
        {"AK60-6", {-12.56, 12.56, -60.0, 60.0, -12.0, 12.0, 0.0, 500.0, 0.0, 5.0}},
        {"AK70-9", {-12.56, 12.56, -30.0, 30.0, -32.0, 32.0, 0.0, 500.0, 0.0, 5.0}},
        {"AK80-9", {-12.56, 12.56, -65.0, 65.0, -18.0, 18.0, 0.0, 500.0, 0.0, 5.0}},
        {"AKE60-8", {-12.56, 12.56, -40.0, 40.0, -15.0, 15.0, 0.0, 500.0, 0.0, 5.0}},
        {"AKE80-8", {-12.56, 12.56, -20.0, 20.0, -35.0, 35.0, 0.0, 500.0, 0.0, 5.0}}
    };

} // namespace motor_specs