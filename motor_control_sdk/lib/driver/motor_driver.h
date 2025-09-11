#pragma once

#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "motor_control_sdk/msgs/motor_msgs.h"

#include "motor_control_sdk/lib/driver/containers.h"
#include "motor_control_sdk/lib/communication/can_bus.h"
#include "motor_control_sdk/lib/communication/protocols/motor_reply.h"

/**
 * @brief Motor Driver control node for interfacing with CubeMars AK Series motors.
 */
class MotorDriver : public rclcpp::Node {
    public:
         /**
         * @brief Construct a Motor Driver object that manages communication with motors over CAN bus.
         * @param options Node options for ROS2 configuration.
         * @param configs A vector of MotorConfig structs defining each motor's settings.
         */
        explicit MotorDriver(const rclcpp::NodeOptions& options, const std::vector<containers::MotorConfig>& motor_configs);

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
        /**
         * @brief A struct to hold all dynamic runtime state for a single motor.
         */
        struct MotorRuntimeState {
            protocol::MotorReply last_reply;
            rclcpp::Time last_reply_time;
            bool enabled = false;
        };

        // Private methods:
        void setup();
        void command_callback(const cubemars_motor_msgs::msg::MotorCommands::ConstSharedPtr msg);
        void state_callback();
        void can_read_loop(std::stop_token token, const std::string& can_interface);

        // Member variables:
        std::vector<containers::MotorConfig> motor_configs_;
        std::map<std::string, std::shared_ptr<CanBus>, std::less<>> can_buses_;
        int publish_rate_us_;

        // Lookup maps:
        std::map<std::string, const containers::MotorConfig*> motor_config_by_name_;
        std::map<std::string, std::map<uint8_t, std::string>, std::less<>> motor_name_by_bus_id_;
        std::map<std::string, MotorRuntimeState> motor_states_;

        // Threading Management
        std::vector<std::jthread> threads_;
        std::mutex mutex_;

        // ROS2 Interface
        rclcpp::Subscription<cubemars_motor_msgs::msg::MotorCommands>::SharedPtr command_subscriber_;
        rclcpp::Publisher<cubemars_motor_msgs::msg::MotorStates>::SharedPtr state_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};