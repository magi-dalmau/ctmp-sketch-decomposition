FROM ubuntu:20.04
RUN apt-get update && apt-get install -y \
    wget \
    git \
    bash-completion \
    build-essential \
    sudo \
    python3-pip \    
    && rm -rf /var/lib/apt/lists/*
    
#Install tamp-core python dependencies
RUN pip3 install matplotlib    

# Install ROS Noetic
RUN apt-get update &&  apt-get install -y \
curl \
sudo \
lsb-release
RUN /bin/bash -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update &&  \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
ros-noetic-ros-base \
python3-catkin-tools \
#ros-noetic-image-transport \
#ros-noetic-image-geometry \
#ros-noetic-pcl-ros \
#ros-noetic-cv-bridge \
ros-noetic-eigen-conversions \
ros-noetic-tf-conversions \
ros-noetic-kdl-parser \
&& rm -rf /var/lib/apt/lists/*

# Prepare ROS workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# Add source to bashrc
RUN echo "" >> ~/.bashrc
RUN echo "## ROS" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc
