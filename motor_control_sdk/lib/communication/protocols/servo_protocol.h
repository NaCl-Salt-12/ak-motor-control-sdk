#pragma once

#include <linux/can.h>
#include <cstdint>
#include <span>

namespace servo_protocol {

    enum class ControlMode : uint32_t {
        SetDutyCycle     = 0,
        SetCurrent       = 1,
        SetCurrentBrake  = 2,
        SetRPM           = 3,
        SetPosition      = 4,
        SetOrigin        = 5,
        SetPositionSpeed = 6,
    };

    namespace internal {
        inline void append_int16_be(std::span<uint8_t> buffer, int16_t value) {
            buffer[0] = (value >> 8) & 0xFF;
            buffer[1] = value & 0xFF;
        }

        inline void append_int32_be(std::span<uint8_t> buffer, int32_t value) {
            buffer[0] = (value >> 24) & 0xFF;
            buffer[1] = (value >> 16) & 0xFF;
            buffer[2] = (value >> 8) & 0xFF;
            buffer[3] = value & 0xFF;
        }
    }

    // Creates a frame for Duty Cycle Mode
    /**
     * @brief Create a duty cycle frame object
     * @param driver_id 
     * @param duty Desired duty cycle (range -1.0 to 1.0)
     * @return can_frame 
     */
    inline can_frame create_duty_cycle_frame(uint8_t driver_id, float duty) {
        constexpr double DUTY_SCALE = 100000.0; // Scaling from docs
        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetDutyCycle) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        internal::append_int32_be(frame.data, static_cast<int32_t>(duty * DUTY_SCALE));
        return frame;
    }

    // Creates a frame for Current Loop Mode
    /**
     * @brief Create a current frame object
     * @param driver_id 
     * @param current Desired current in Amps
     * @return can_frame 
     */
    inline can_frame create_current_frame(uint8_t driver_id, float current) {
        constexpr double CURRENT_SCALE = 1000.0; // Scaling from docs
        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetCurrent) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        internal::append_int32_be(frame.data, static_cast<int32_t>(current * CURRENT_SCALE));
        return frame;
    }

    // Creates a frame for Current Brake Mode
    /**
     * @brief Create a current brake frame object
     * @param driver_id 
     * @param current 
     * @return can_frame 
     */
    inline can_frame create_current_brake_frame(uint8_t driver_id, float current) {
        constexpr double CURRENT_SCALE = 1000.0; // Scaling from docs
        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetCurrentBrake) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        internal::append_int32_be(frame.data, static_cast<int32_t>(current * CURRENT_SCALE));
        return frame;
    }

    // Creates a frame for Velocity Loop Mode
    /**
     * @brief Create a rpm frame object
     * @param driver_id 
     * @param velocity Desired speed in electrical RPM 
     * @return can_frame 
     */
    inline can_frame create_velocity_frame(uint8_t driver_id, float velocity) {
        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetRPM) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        internal::append_int32_be(frame.data, static_cast<int32_t>(velocity));
        return frame;
    }

    // Creates a frame for Position Loop Mode
    /**
     * @brief Create a position frame object
     * @param driver_id 
     * @param position 
     * @return can_frame 
     */
    inline can_frame create_position_frame(uint8_t driver_id, float position) {
        constexpr double POSITION_SCALE = 10000.0;  // Scaling from docs
        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetPosition) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        internal::append_int32_be(frame.data, static_cast<int32_t>(position * POSITION_SCALE));
        return frame;
    }

    // Creates a frame for setting the motor's origin position
    /**
     * @brief Create a set origin frame object
     * @param driver_id 
     * @param set_permanent 
     * @return can_frame 
     */
    inline can_frame create_set_origin_frame(uint8_t driver_id, bool set_permanent) {
        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetOrigin) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 1;
        frame.data[0] = set_permanent ? 1 : 0; // 1 for permanent, 0 for temporary
        return frame;
    }

    // Creates a frame for Position-Velocity Loop Mode.
    /**
     * @brief Create a position speed frame object
     * @param driver_id 
     * @param position Desired position in degrees
     * @param velocity Desired speed in electrical RPM
     * @param acceleration Desired acceleration in electrical RPM/s^2
     * @return can_frame 
     */
    inline can_frame create_position_speed_frame(uint8_t driver_id, float position, int16_t velocity, int16_t acceleration) {
        constexpr double POSITION_SCALE = 10000.0; // Scaling from docs

        can_frame frame{};
        frame.can_id = driver_id | (static_cast<uint32_t>(ControlMode::SetPositionSpeed) << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;
        
        // The manual states that the sent speed and acceleration values are scaled by 10
        const int16_t scaled_velocity = velocity / 10;
        const int16_t scaled_acceleration = acceleration / 10;

        auto buffer_span = std::span(frame.data);
        internal::append_int32_be(buffer_span.subspan<0, 4>(), static_cast<int32_t>(position * POSITION_SCALE));
        internal::append_int16_be(buffer_span.subspan<4, 2>(), scaled_velocity);
        internal::append_int16_be(buffer_span.subspan<6, 2>(), scaled_acceleration);

        return frame;
    }

} // namespace servo_protocol