# Use ROS2 Foxy base image
FROM ros:foxy-ros-base

# Set the timezone to avoid prompts during installation
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    build-essential \
    python3-colcon-common-extensions \
    git \
    ros-foxy-ament-cmake \
    ros-foxy-std-msgs \
    ros-foxy-builtin-interfaces \
    ros-foxy-rclcpp \
    ros-foxy-dynamixel-sdk \
    # ros-foxy-turtlebot3 \
    # ros-foxy-turtlebot3-simulations \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    # && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV TURTLEBOT3_MODEL=burger

# Create workspace directory
WORKDIR /root/asti2024_ws

# Set up entrypoint
COPY docker-entry.sh /docker-entry.sh
RUN chmod +x /docker-entry.sh

ENTRYPOINT ["/docker-entry.sh"]
CMD ["bash"]


# docker run your-image
# docker run your-image ros2 run some_package some_node