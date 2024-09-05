# Use the official Ubuntu 22.04 image as a base
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
RUN apt-get update && \
    apt-get install -y \
    locales \
    build-essential \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# Add the ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /etc/apt/trusted.gpg.d/ros.gpg > /dev/null

# Add the ROS 2 repository
RUN echo "deb [signed-by=/etc/apt/trusted.gpg.d/ros.gpg] http://repo.ros2.org/ubuntu/main jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# Update apt and install ROS 2 Humble packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-desktop \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Setup the environment for ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /ros2_ws

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]