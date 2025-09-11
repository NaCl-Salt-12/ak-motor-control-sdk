#pragma once

#include <string>
#include <cstdint>

#include "motor_control_sdk/lib/motor_specs.h"


namespace containers {

    /**
     * @brief Configuration structure for a motor.
     * @param name A unique name for the motor.
     * @param id The motor's ID on the CAN bus (0-255).
     * @param interface The CAN interface to use (e.g., "can0").
     * @param limits The motor's operational limits as per motor_specs.
     */
    struct MotorConfig {
        std::string name;
        uint8_t id;
        std::string interface;
        motor_specs::MotorLimits limits;
    };

} // namespace containers
