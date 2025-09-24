#include "rclcpp/rclcpp.hpp"
#include "motor_control_sdk/lib/driver/motor_driver.h"
#include "motor_control_sdk/lib/driver/containers.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Example motor configuration - users would customize this
    std::vector<containers::MotorConfig> motor_configs = {
        // Add your motor configurations here
        // Example:
        // {
        //     .name = "motor1",
        //     .id = 1,
        //     .interface = "can0",
        //     .limits = {/* your limits */}
        // }
    };

    // Create the motor driver node
    auto node = std::make_shared<MotorDriver>(rclcpp::NodeOptions(), motor_configs);

    RCLCPP_INFO(node->get_logger(), "Motor Driver Node started");

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error in motor driver: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
