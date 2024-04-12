#! /bin/bash

source /opt/ros/foxy/setup.bash
source install/setup.bash

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=30

#!/bin/bash

screen -S cam_pub -dm bash -c "source /opt/ros/foxy/setup.bash; source install/setup.bash; export ROS_LOCALHOST_ONLY=0; export ROS_DOMAIN_ID=30; ros2 run final cam_pub; exec bash"
screen -S lector_archivo -dm bash -c "source /opt/ros/foxy/setup.bash; source install/setup.bash; export ROS_LOCALHOST_ONLY=0; export ROS_DOMAIN_ID=30; ros2 run pruebas lector_archivo; exec bash"
screen -S lector_cmd_vel -dm bash -c "sudo su; source /opt/ros/foxy/setup.bash; source install/setup.bash; export ROS_LOCALHOST_ONLY=0; export ROS_DOMAIN_ID=30; ros2 run pruebas lector_cmd_vel; exec bash"
screen -S camara_sub -dm bash -c "source /opt/ros/foxy/setup.bash; source install/setup.bash; export ROS_LOCALHOST_ONLY=0; export ROS_DOMAIN_ID=30; ros2 run final camara_sub; exec bash"