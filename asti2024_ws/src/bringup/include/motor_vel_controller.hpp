#ifndef MOTOR_CONTROLLER_HPP_
#define MOTOR_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/msg/set_velocity.hpp"
#include "custom_interfaces/srv/get_velocity.hpp"
#include "custom_interfaces/msg/set_position.hpp"
#include "custom_interfaces/srv/get_position.hpp"

class MotorController : public rclcpp::Node {
public:
   using Twist = geometry_msgs::msg::Twist;
   using Float32 = std_msgs::msg::Float32;
   using GetVelocity = custom_interfaces::srv::GetVelocity;
   using SetPosition = custom_interfaces::msg::SetPosition;
   using GetPosition = custom_interfaces::srv::GetPosition;

   MotorController();
   virtual ~MotorController();

private:
   rclcpp::Subscription<Twist>::SharedPtr cmd_vel_subscriber_;
   rclcpp::Subscription<SetPosition>::SharedPtr tool_pos_subscriber_;
   rclcpp::Service<GetVelocity>::SharedPtr get_velocity_server_;
   rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

   int current_velocity;
   int present_position;
};

#endif // MOTOR_CONTROLLER_HPP_
