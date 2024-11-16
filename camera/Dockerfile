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
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble packages
RUN apt-get update && \
    apt-get install -y ros-humble-desktop && \
    apt-get install -y ros-dev-tools && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install ros-humble-ament-cmake && \
    apt-get install ros-humble-cv-bridge && \
    apt-get install ros-humble-rviz2 && \
    apt-get install ros-humble-image-transport && \
    apt-get install libqt5gui5  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install OpenCV Python packages
RUN pip3 install opencv-contrib-python==4.10.0.84 opencv-python==4.10.0.84

# Setup the environment for ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN rosdep init && rosdep update

# Create and set the working directory
WORKDIR /ros2_ws

COPY /run.sh /ros2_ws/run.sh
COPY /src /ros2_ws/src
RUN chmod +x ./run.sh

# test camera
COPY camera_test.py /camera_test/camera_test.py

# Set bash as the entrypoint
ENTRYPOINT ["/bin/bash"]
