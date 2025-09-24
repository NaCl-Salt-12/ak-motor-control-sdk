#include "motor_driver.h"

#include <exception>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <chrono>
#include <atomic>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "motor_control_sdk/msg/motor_command.hpp"
#include "motor_control_sdk/msg/motor_commands.hpp"
#include "motor_control_sdk/msg/motor_state.hpp"
#include "motor_control_sdk/msg/motor_states.hpp"

#include "motor_control_sdk/lib/driver/containers.h"
#include "motor_control_sdk/lib/communication/can_bus.h"
#include "motor_control_sdk/lib/communication/protocols/motor_reply.h"
#include "motor_control_sdk/lib/communication/protocols/force_control_protocol.h"

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
    
    // Signal threads to stop
    stop_threads_ = true;
    
    // Wait for all threads to finish
    for (auto& thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

void MotorDriver::setup() {
    RCLCPP_INFO(this->get_logger(), "Setting up MotorDriver with %zu motor configurations.", motor_configs_.size());

    // Validate and organize motor configurations
    for (const auto& config : motor_configs_) {
        if (motor_config_by_name_.count(config.name)) {
            std::stringstream ss;
            ss << "Duplicate motor name detected: " << config.name;
            throw std::runtime_error(ss.str());
        }

        if (motor_name_by_bus_id_[config.interface].count(config.id)) {
            std::stringstream ss;
            ss << "Duplicate motor ID " << config.id << " on interface " << config.interface;
            throw std::runtime_error(ss.str());
        }

        motor_config_by_name_[config.name] = &config;
        motor_name_by_bus_id_[config.interface][config.id] = config.name;
        motor_states_.try_emplace(config.name);
    }

    // Initialize CAN buses based on motor configurations
    for (const auto& config : motor_configs_) {
        if (can_buses_.find(config.interface) == can_buses_.end()) {
            RCLCPP_INFO(this->get_logger(), "Initializing CAN bus on interface: %s", config.interface.c_str());
            try {
                auto bus = std::make_shared<CanBus>(config.interface);

                // TODO(jeh15): This is something we need to know.
                bus->set_read_timeout(2ms);

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
        [this](const motor_control_sdk::msg::MotorCommands::ConstSharedPtr msg) {
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
    for (const auto& [name, bus_ptr] : can_buses_) {
        threads_.emplace_back(
        [this](std::string name) {
            this->can_read_loop(name);
        }, 
        std::string(name));
    }
}

/**
 * @brief Callback for receiving motor commands from the ROS network.
 * @param msg The received MotorCommands message.
 */
void MotorDriver::command_callback(const motor_control_sdk::msg::MotorCommands::ConstSharedPtr msg) {
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
void MotorDriver::can_read_loop(const std::string& can_interface) {
    RCLCPP_INFO(this->get_logger(), "CAN reader thread started for %s.", can_interface.c_str());

    auto can_bus_it = can_buses_.find(can_interface);
    if (can_bus_it == can_buses_.end()) {
        RCLCPP_FATAL(this->get_logger(), "Invalid CAN interface name passed to reader thread: %s", can_interface.c_str());
        return;
    }
    const auto& can_bus = can_bus_it->second;

    auto bus_map_it = motor_name_by_bus_id_.find(can_interface);
    if (bus_map_it == motor_name_by_bus_id_.end()) {
        RCLCPP_FATAL(this->get_logger(), "Invalid CAN interface name passed to reader thread: %s", can_interface.c_str());
        return;
    }
    const auto& id_map = bus_map_it->second;

    while (!stop_threads_) {
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
                    can_interface.c_str()
                );
                std::stringstream ss;
                ss << "Unknown motor ID " << id << " on CAN bus " << can_interface;
                throw std::runtime_error(ss.str());
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
        states_msg.motor_states.reserve(motor_states_.size());
        for (const auto& [name, state] : motor_states_) {
            motor_control_sdk::msg::MotorState state_msg;

            state_msg.name = name;
            state_msg.position = state.last_reply.position;
            state_msg.velocity = state.last_reply.velocity;
            state_msg.current = state.last_reply.current;
            state_msg.temperature = state.last_reply.temperature;
            state_msg.error_code = state.last_reply.error_code;

            states_msg.motor_states.push_back(state_msg);
        }
    }
    
    // Publish the consolidated message
    state_publisher_->publish(states_msg);
}
