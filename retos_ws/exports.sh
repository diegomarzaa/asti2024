#! /bin/bash

source /opt/ros/foxy/setup.bash
source install/setup.bash

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=30

#!/bin/bash

lxterminal -e '/bin/bash -c "source /opt/ros/foxy/setup.bash;
                source install/setup.bash; export ROS_LOCALHOST_ONLY=0;
                export ROS_DOMAIN_ID=30;
                exec bash"'
lxterminal -e '/bin/bash -c "source /opt/ros/foxy/setup.bash;
                source install/setup.bash;
                export ROS_LOCALHOST_ONLY=0;
                export ROS_DOMAIN_ID=30;
                exec bash"'
lxterminal -e '/bin/bash -c "source /opt/ros/foxy/setup.bash;
                source install/setup.bash;
                export ROS_LOCALHOST_ONLY=0;
                export ROS_DOMAIN_ID=30;
                exec bash"'
lxterminal -e '/bin/bash -c "source /opt/ros/foxy/setup.bash;
                source install/setup.bash;
                export ROS_LOCALHOST_ONLY=0;
                export ROS_DOMAIN_ID=30;
                exec bash"'