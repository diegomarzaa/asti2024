///////////////////////////////////////////////////////////////////////////////
//                     *** DYNAMIXEL MOTOR ROS2 NODE ***
//
//                         * Enable USB port access *
// >> sudo usermod -aG dialout <linux_account>
//                           (Restart your computer)
//
//                               * Start node *
// >> ros2 run bringup motor_controller
//
// if it doesnt work, try:
// >> ros2 run bringup motor_controller <device_name>
//         - Optional argument (default: /dev/ttyUSB0)
//         - Use ls /dev/ttyUSB* to find the correct device name
//         - Use sudo chmod 777 /dev/ttyUSB0 to give permissions
//
//             * Send Twist messages to /cmd_vel topic *
// >> ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
// 
//            * Send SetPosition messages to /tool_pos topic *
// >> ros2 topic pub -1 /tool_pos custom_interfaces/msg/SetPosition "{position: 90, id: 3}"
///////////////////////////////////////////////////////////////////////////////

#include <cstdio> // Dynamixel SDK
#include <memory> // Dynamixel SDK
#include <string> // Dynamixel SDK

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/srv/get_velocity.hpp"
#include "custom_interfaces/srv/get_position.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcutils/cmdline_parser.h" // Dynamixel SDK
#include "std_msgs/msg/float32.hpp"


#include "motor_vel_controller.hpp"

// Control table for Dynamixel X Series
#define ADDR_OPERATING_MODE 11 // 1 for velocity control | 3 for position control
#define ADDR_TORQUE_ENABLE 64 // 0 for torque off | 1 for torque on
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default settings
#define BAUDRATE 115200 // 57600 Default baudrate
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0" // ls /dev/ttyUSB* to find the correct device name

// Robot parameters
#define VELOCITY_UNIT 0.229 // rpm | See https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/#velocity-limit for more details
#define WHEEL_DIAMETER 0.068 // meters
#define WHEEL_SEPARATION 0.172 // meters

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Constants
constexpr double pi = 3.141592653589793;

// Variables for motor control
uint8_t id_herramienta = 3;
uint32_t goal_position = 0;
int32_t right_wheel_velocity = 0;
int32_t left_wheel_velocity = 0;

// Unit that allows the program to convert desired robot velocity to motor velocity units
const double distance_unit = 1 / (pi * VELOCITY_UNIT * WHEEL_DIAMETER / 60);

// Error handling
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;


MotorController::MotorController()
: Node("motor_controller") {

   RCLCPP_INFO(this->get_logger(), "Motor Controller node started");
   
   this->declare_parameter("qos_depth", 10);
   int8_t qos_depth = 0;
   this->get_parameter("qos_depth", qos_depth);

   const auto QOS_RKL10V = // Defines QoS
   rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

   // ╔═════════════════════════════╗
   // ║  CMD_VEL TO MOTOR_MOVEMENT  ║
   // ╚═════════════════════════════╝
    
   cmd_vel_subscriber_ =
      this->create_subscription<Twist>(
      "cmd_vel",
      QOS_RKL10V,
      [this](const Twist::SharedPtr msg) -> void {
         uint8_t dxl_error = 0;

         // Read linear and angular goal velocities
         double linear_velocity = msg->linear.x;
         double angular_velocity = msg->angular.z;

         // Calculate right and left wheel velocities
         if (angular_velocity == 0) {
            right_wheel_velocity = linear_velocity * distance_unit;
            left_wheel_velocity = linear_velocity * distance_unit;
         } else {
            /*if (angular_velocity > 0) {
               right_wheel_velocity = (angular_velocity * (linear_velocity + WHEEL_SEPARATION / 2)) * distance_unit;
               left_wheel_velocity = (angular_velocity * (linear_velocity - WHEEL_SEPARATION / 2)) * distance_unit;
            } else {
               right_wheel_velocity = (-angular_velocity * (linear_velocity - WHEEL_SEPARATION / 2)) * distance_unit;
               left_wheel_velocity = (-angular_velocity * (linear_velocity + WHEEL_SEPARATION / 2)) * distance_unit;
            }*/
        if (angular_velocity > 0) {
            right_wheel_velocity = ((2*linear_velocity + WHEEL_SEPARATION*angular_velocity) / 2) * distance_unit;
               left_wheel_velocity = ((2*linear_velocity - WHEEL_SEPARATION*angular_velocity) / 2) *  distance_unit;
            } else {
               right_wheel_velocity = ((2*linear_velocity + WHEEL_SEPARATION*angular_velocity) / 2) * distance_unit;
               left_wheel_velocity = ((2*linear_velocity - WHEEL_SEPARATION*angular_velocity) / 2) *  distance_unit;
         }
         }

         // Send goal velocity (4 bytes) to each one of the DYNAMIXELs
         dxl_comm_result =
         packetHandler->write4ByteTxRx(
            portHandler, 
            1, // Right wheel ID
            ADDR_GOAL_VELOCITY, 
            right_wheel_velocity, 
            &dxl_error
         );

         if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
         } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
         } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", 1, right_wheel_velocity);
         }

         dxl_comm_result =
         packetHandler->write4ByteTxRx(
            portHandler, 
            2, // Left wheel ID
            ADDR_GOAL_VELOCITY, 
            left_wheel_velocity, 
            &dxl_error
         );

         if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
         } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
         } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", 2, left_wheel_velocity);
         }
      }
   );

   // ╔═════════════════════════════╗
   // ║  TOOL_POS TO MOTOR_MOVEMENT ║
   // ╚═════════════════════════════╝

   tool_pos_subscriber_ =
      this->create_subscription<SetPosition>(
      "tool_pos",
      QOS_RKL10V,
      [this](const SetPosition::SharedPtr msg) -> void {
         uint8_t dxl_error = 0;

         uint32_t goal_degrees = (unsigned int) msg->position;   // Convert int32 -> uint32
         uint32_t goal_units = goal_degrees * 11.375;    // Convert normal degrees provided by the user to Dynamixel units

         // Write goal_units (4 bytes) to the DYNAMIXEL
         dxl_comm_result =
         packetHandler->write4ByteTxRx(
            portHandler, 
            id_herramienta, // Tool ID set at the beginning of the program
            ADDR_GOAL_POSITION,
            goal_units,     // Goal position
            &dxl_error
         );

         if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
         } else if (dxl_error != 0) {
            RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
         } else {
            RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", id_herramienta, goal_units);
         }
      }
   );

   // ╔═════════════════════════════╗
   // ║  GET CURRENT VELOCITY       ║
   // ╚═════════════════════════════╝

   // Defines a service to get the motor's current velocity
   auto get_current_velocity =
      [this](
      const std::shared_ptr<GetVelocity::Request> request,
      std::shared_ptr<GetVelocity::Response> response) -> void {
         
         // Read current velocity (4 bytes)
         dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, 
            (uint8_t) request->id, 
            ADDR_PRESENT_VELOCITY, 
            reinterpret_cast<uint32_t*>(&current_velocity),
            &dxl_error
         );

         RCLCPP_INFO(
            this->get_logger(),
            "Get [ID: %d] [Current Velocity: %d]",
            request->id,
            current_velocity
         );

         response->velocity = current_velocity;
      };
   get_velocity_server_ = create_service<GetVelocity>("get_velocity", get_current_velocity);

   // ╔═════════════════════════════╗
   // ║  GET CURRENT POSITION       ║
   // ╚═════════════════════════════╝

   auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

MotorController::~MotorController() { RCLCPP_INFO(this->get_logger(), "Motor Controller node stopped"); }

void setupDynamixel(uint8_t dxl_id) {

   // ----- SET VELOCITY MODE TO ALL MOTORS
   dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, 
      dxl_id, 
      ADDR_OPERATING_MODE, 
      1, 
      &dxl_error
   );

   if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to set Velocity Control mode.");
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to set Velocity Control mode.");
   }

   // ------ THEN ONLY SET POSITION MODE TO THE TOOL MOTOR
   dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, 
      id_herramienta,
      ADDR_OPERATING_MODE, 
      3,
      &dxl_error
   );

   // -------- Enable Torque so the motor can move (EEPROM will be locked)
   // IMPORTANT: Torque must be disabled to change the operating mode
   dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, 
      dxl_id, 
      ADDR_TORQUE_ENABLE, 
      1, 
      &dxl_error
   );

   if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to enable Torque.");
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to enable Torque.");
   }
}

int main(int argc, char * argv[]) {
   
   // Get custom device name
   const char* deviceName = DEFAULT_DEVICE_NAME;
   if (argc > 1) {
      deviceName = argv[1]; 
   }
   std::cout << "Using device: " << deviceName << std::endl;

   portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
   packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
   
   // Open Serial Port
   dxl_comm_result = portHandler->openPort();
   if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to open the port!");
      return -1;
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to open the port.");
   }
   
   // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
   dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
   if (dxl_comm_result == false) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_controller"), "Failed to set the baudrate!");
      return -1;
   } else {
      RCLCPP_INFO(rclcpp::get_logger("motor_controller"), "Succeeded to set the baudrate.");
   }
   
   // Initialize Motors with the correct operating mode for each one,
   // and enable Torque for all of them
   setupDynamixel(BROADCAST_ID);
   
   // Keep the node running until closed
   rclcpp::init(argc, argv);
   auto motorcontroller = std::make_shared<MotorController>();
   rclcpp::spin(motorcontroller);
   
   // On shutdown, disable Torque of DYNAMIXEL
   rclcpp::shutdown();
   packetHandler->write1ByteTxRx(
      portHandler,
      BROADCAST_ID,
      ADDR_TORQUE_ENABLE,
      0,
      &dxl_error
   );
   
   return 0;
}
