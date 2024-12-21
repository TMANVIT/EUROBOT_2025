# Use the official Ubuntu 22.04 image as a base
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

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
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble packages
RUN apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-desktop && \
    apt-get install -y ros-dev-tools && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install ros-$ROS_DISTRO-ament-cmake && \
    apt-get install -y ros-$ROS_DISTRO-micro-ros-msgs && \
    apt-get install -y ros-$ROS_DISTRO-robot-localization && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create and set the working directory
WORKDIR /ros2_ws

# Copy and set permissions for the run script
COPY /run.sh /ros2_ws/run.sh

RUN mkdir src \
    && git -C src clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git \
    && git -C src clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_msgs.git \
    &&  rm -rf log/ build/ src/* \
    &&  rm -rf /var/lib/apt/lists/*

# Setup the environment for ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN sudo rosdep fix-permissions && \
    rosdep init && rosdep update

RUN chmod +x ./run.sh
RUN ./run.sh

# Set bash as the entrypoint
ENTRYPOINT ["/bin/bash"]
