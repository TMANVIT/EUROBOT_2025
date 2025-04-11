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
    cmake \
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
    apt-get install -y ros-$ROS_DISTRO-cv-bridge && \
    apt-get install -y ros-$ROS_DISTRO-rviz2 && \
    apt-get install -y ros-$ROS_DISTRO-image-transport && \
    apt-get install -y ros-$ROS_DISTRO-joint-state-publisher-gui && \
    apt-get install -y ros-$ROS_DISTRO-xacro && \
    apt-get install -y ros-$ROS_DISTRO-robot-localization && \
    apt-get install -y ros-$ROS_DISTRO-navigation2 && \
    apt-get install -y ros-$ROS_DISTRO-nav2-bringup && \
    apt-get install -y v4l-utils && \
    apt-get install -y ros-$ROS_DISTRO-v4l2-camera && \
    apt-get install -y ros-$ROS_DISTRO-image-transport-plugins && \
    apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    apt-get install -y ros-$ROS_DISTRO-nav2-smac-planner && \
    apt-get install -y libqt5gui5  && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /packages
COPY ./src/python .
RUN bash -c "pip install ./cvFunctions"  
# RUN bash -c "pip install -r ./requirements.txt"

# Set the working directory and clone Cyclone DDS repository
WORKDIR /opt
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds.git

# Build and install Cyclone DDS
WORKDIR /opt/cyclonedds
RUN mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    cmake --build . --target install

# Create and set the working directory
WORKDIR /ros2_ws
COPY ./cyclonedds.xml /ros2_ws/cyclonedds.xml

# Setup the environment for ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN sudo rosdep fix-permissions && \
    rosdep init && rosdep update

# Copy and set permissions for the run script
COPY ./docker/run.sh /ros2_ws/run.sh
RUN chmod +x ./run.sh

# Set bash as the entrypoint
ENTRYPOINT ["/bin/bash"]