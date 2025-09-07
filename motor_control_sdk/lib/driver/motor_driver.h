#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

/**
 * @brief Motor Driver control node for interfacing with CubeMars AK Series motors.
 */
class MotorDriver : public rclcpp::Node {
    public:
         /**
         * @brief Construct a new Motor Driver object
         */
        MotorDriver();

        /**
         * @brief Destroy the Motor Driver object, ensuring proper cleanup of resources.
         */
        ~MotorDriver();

        // Disable copy and move semantics to prevent accidental duplication
        MotorDriver(const MotorDriver&) = delete;
        MotorDriver& operator=(const MotorDriver&) = delete;
        MotorDriver(MotorDriver&&) = delete;
        MotorDriver& operator=(MotorDriver&&) = delete;

    private:
        // Private member variables for motor control

        // ROS2 subscription:
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;



        std::mutex motor_mutex_;
        std::thread motor_thread_;
        bool running_;
        std::optional<std::string> motor_id_;
};