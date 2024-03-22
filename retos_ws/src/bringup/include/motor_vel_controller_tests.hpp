#ifndef MOTOR_CONTROLLER_HPP_
#define MOTOR_CONTROLLER_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/msg/set_velocity.hpp"
#include "custom_interfaces/srv/get_velocity.hpp"


class MotorController : public rclcpp::Node {
public:
   using Twist = geometry_msgs::msg::Twist;
   using GetVelocity = custom_interfaces::srv::GetVelocity;

   MotorController();
   virtual ~MotorController();

private:
   rclcpp::Subscription<Twist>::SharedPtr cmd_vel_subscriber_;
   rclcpp::Service<GetVelocity>::SharedPtr get_velocity_server_;

   int current_velocity;
};

#endif // MOTOR_CONTROLLER_HPP_
