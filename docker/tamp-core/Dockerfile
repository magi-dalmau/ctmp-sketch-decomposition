FROM eut-moveit:dev-noetic-task-constructor
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \    
    python3-pip     
   
    
#Install tamp-core python dependencies
RUN pip3 install matplotlib    

#Install ros tamp-core ros dependencies (not included in the basic eut-ros-noetic image)
RUN apt-get update &&  \
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
ros-noetic-kdl-parser 

#Download Tiago related packages
WORKDIR /ros_ws/src
RUN git clone https://github.com/pal-robotics/tiago_robot.git
RUN git clone https://github.com/pal-robotics/tiago_description_calibration.git
RUN git clone https://github.com/pal-robotics/pmb2_robot.git
RUN git clone https://github.com/pal-robotics/hey5_description.git
RUN git clone https://github.com/pal-robotics/pal_gripper.git
RUN git clone https://github.com/pal-robotics/tiago_moveit_config.git 

#Build the updated ros workspace
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"

# Remove apt cache
RUN rm -rf /var/lib/apt/lists/*


