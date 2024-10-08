# Use the official Ubuntu 22.04 image as a base
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

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
    python3-pip \
    software-properties-common && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Add the ROS 2 GPG key and repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /etc/apt/trusted.gpg.d/ros.gpg > /dev/null && \
    echo "deb [signed-by=/etc/apt/trusted.gpg.d/ros.gpg] http://repo.ros2.org/ubuntu/main jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 Humble packages
RUN apt-get update && \
    apt-get install -y ros-humble-desktop && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Setup the environment for ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create and set the working directory
WORKDIR /ros2_ws

# Copy and set permissions for the run script
COPY ./run.sh ./ros2_ws/run.sh
RUN chmod +x ./ros2_ws/run.sh

# Run the setup script (if you need it to run in build time)
RUN ./ros2_ws/run.sh

# Set bash as the entrypoint
ENTRYPOINT ["/bin/bash"]
