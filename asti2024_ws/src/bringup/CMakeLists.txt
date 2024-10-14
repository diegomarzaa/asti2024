cmake_minimum_required(VERSION 3.5)
project(bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# add_executable(motor_controller src/motor_controller.cpp)
# ament_target_dependencies(motor_controller 
#   rclcpp
#   dynamixel_sdk
#   custom_interfaces
# )

add_executable (motor_vel_controller src/motor_vel_controller.cpp)
ament_target_dependencies(motor_vel_controller
  rclcpp
  dynamixel_sdk
  custom_interfaces
  geometry_msgs
  std_msgs
)

install(TARGETS
  motor_vel_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
