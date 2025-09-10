#include "motor_driver.h"

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "motor_control_sdk/msg/motor_commands.hpp"
#include "motor_control_sdk/msg/motor_states.hpp"

#include "motor_control_sdk/lib/communication/can_bus.h"
#include "motor_control_sdk/lib/driver/containers.h"
#include "motor_control_sdk/lib/communication/protocol/motor_reply.h"

#include "motor_control_sdk/force_control_protocol.h"

using namespace std::chrono_literals;


MotorDriver::MotorDriver(const rclcpp::NodeOptions& options, const std::vector<containers::MotorConfig>& motor_configs)
    : Node("motor_driver", options), motor_configs_(motor_configs) {
    
    // Declare parameters:
    declare_parameter("publish_rate_us", 2000);
    this->publish_rate_us_ = this->get_parameter("publish_rate_us").as_int();

    // Initialize internal structures, CAN buses, and ROS interfaces
    this->setup();
}

MotorDriver::~MotorDriver() {
    RCLCPP_INFO(this->get_logger(), "Shutting down MotorDriver node.");
}

void MotorDriver::setup() {
    RCLCPP_INFO(this->get_logger(), "Setting up MotorDriver with %zu motor configurations.", motor_configs_.size());

    // Validate and organize motor configurations
    for (const auto& config : motor_configs_) {
        if (motor_config_by_name_.count(config.name))
            throw std::runtime_error("Duplicate motor name detected: " + config.name);

        if (motor_name_by_bus_id_[config.interface].count(config.id))
            throw std::runtime_error("Duplicate motor ID " + std::to_string(config.id) + " on interface " + config.interface);

        motor_config_by_name_[config.name] = &config;
        motor_name_by_bus_id_[config.interface][config.id] = config.name;
        motor_states_[config.name] = MotorRuntimeState{};
    }

    // Initialize CAN buses based on motor configurations
    for (const auto& config : motor_configs_) {
        if (can_buses_.find(config.interface) == can_buses_.end()) {
            RCLCPP_INFO(this->get_logger(), "Initializing CAN bus on interface: %s", config.interface.c_str());
            try {
                auto bus = std::make_shared<CanBus>(config.interface);

                // TODO(jeh15): This is something we need to know.
                bus->set_read_timeout(100ms);
                //

                can_buses_[config.interface] = bus;
            } catch (const std::system_error& e) {
                RCLCPP_FATAL(this->get_logger(), "Failed to initialize CAN bus '%s': %s", config.interface.c_str(), e.what());
                throw;
            }
        }
    }

    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    command_subscriber_ = this->create_subscription<motor_control_sdk::msg::MotorCommands>(
        "/motor_commands",
        qos_profile,
        [this](const motor_control_sdk::msg::MotorCommands::SharedPtr msg) {
            this->command_callback(msg);
        }
    );

    state_publisher_ = this->create_publisher<motor_control_sdk::msg::MotorStates>(
        "/motor_states",
        qos_profile
    );

    timer_ = create_wall_timer(
        std::chrono::microseconds(publish_rate_us_),
        [this]() { this->state_callback(); }
    );

    // Dedicated thread for CAN Reading:
    for (auto const& [name, bus_ptr] : can_buses_)
        threads_.emplace_back(&MotorDriver::can_read_loop, this, name);

}

/**
 * @brief Callback for receiving motor commands from the ROS network.
 * @param msg The received MotorCommands message.
 */
void MotorDriver::command_callback(const motor_control_sdk::msg::MotorCommands::SharedPtr msg) {
    for (const auto& command : msg->motor_commands) {
        auto it = motor_config_by_name_.find(command.name);
        if (it == motor_config_by_name_.end()) {
            RCLCPP_FATAL(this->get_logger(), "Received command for unknown motor: '%s'.", command.name.c_str());
            throw std::runtime_error("Unknown motor name: " + command.name);
        }

        // Get the Config and CAN Bus
        const auto& config = *it->second;
        auto& can_bus = can_buses_.at(config.interface);

        can_frame frame;
        if (command.enable) {
            protocol::force_control::ForceControlCommand cmd = {
                .position_setpoint = static_cast<float>(command.position_setpoint),
                .velocity_setpoint = static_cast<float>(command.velocity_setpoint),
                .feedforward_torque = static_cast<float>(command.feedforward_torque),
                .kp = static_cast<float>(command.kp),
                .kd = static_cast<float>(command.kd)
            };
            frame = protocol::force_control::create_force_control_frame(config.id, cmd, config.limits);
        }
        else {
            frame = protocol::force_control::create_force_control_frame(config.id, {}, config.limits);
        }
        
        // Write the frame to the bus
        can_bus->write_frame(frame);
        
        // Lock shared state
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motor_states_[command.name].enabled = command.enable;
        }
    }
}

/**
 * @brief Main loop for a dedicated thread that reads CAN frames from a single bus.
 */
void MotorDriver::can_read_loop(std::stop_token token, std::string_view can_interface) {
    RCLCPP_INFO(this->get_logger(), "CAN reader thread started for %s.", can_interface.c_str());
    const auto& can_bus = can_buses_.at(can_interface);
    const auto& id_map = motor_name_by_bus_id_.at(can_interface);

    while (!token.stop_requested()) {
        auto result = can_bus->read_frame();

        if (result.has_value()) {
            const auto& frame = result.value();
            const uint8_t id = frame.can_id & 0xFF;

            auto it = id_map.find(id);
            if (it != id_map.end()) {
                const std::string& name = it->second;
                auto reply = protocol::unpack_reply_frame(frame.data);
                
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    motor_states_[name].last_reply = reply;
                    motor_states_[name].last_reply_time = this->get_clock()->now();
                }
            }
            else {
                RCLCPP_FATAL(
                    this->get_logger(),
                    "Received CAN frame from unknown motor ID %u on interface '%s'.",
                    id,
                    std::string(can_interface).c_str()
                );
                throw std::runtime_error("Unknown motor ID on CAN bus");
            }
        } 
        else if (result.error() != std::errc::timed_out) {
            RCLCPP_WARN(
                this->get_logger(), "CAN read error on %s: %s",
                can_interface.c_str(),
                std::make_error_code(result.error()).message().c_str()
            );
        }
    }
    RCLCPP_INFO(this->get_logger(), "CAN reader thread stopping for %s.", can_interface.c_str());
}

/**
 * @brief Timer callback to periodically publish the state of all motors.
 */
void MotorDriver::state_callback() {
    motor_control_sdk::msg::MotorStates states_msg;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        states_msg.states.reserve(motor_states_.size());
        for (const auto& [name, state] : motor_states_) {
            states_msg.states.push_back(
                motor_control_sdk::msg::MotorState{
                    .name = name,
                    .position = state.last_reply.position,
                    .velocity = state.last_reply.velocity,
                    .current = state.last_reply.current,
                    .temperature = state.last_reply.temperature,
                    .error_code = state.last_reply.error_code
                });
        }
    }
    
    // Publish the consolidated message
    state_publisher_->publish(states_msg);
}
