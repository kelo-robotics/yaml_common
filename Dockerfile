FROM ros:noetic

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get -qq update > /dev/null && \
    apt-get -yqq install git \
                         python3-catkin-tools \
                         ros-$ROS_DISTRO-catkin \
                         ros-$ROS_DISTRO-tf \
                         ros-$ROS_DISTRO-tf2-geometry-msgs \
                         libyaml-cpp-dev > /dev/null && \
    apt-get clean > /dev/null

WORKDIR /workspace/catkin_ws/src

# Clone dependencies
RUN git clone https://github.com/kelo-robotics/geometry_common.git

# Copy the yaml_common source code to the docker container
WORKDIR /workspace/catkin_ws/src/yaml_common
ADD . /workspace/catkin_ws/src/yaml_common/

# Compile the ROS catkin workspace
# RUN cd /workspace/catkin_ws && \
#     /ros_entrypoint.sh catkin build --no-status

# Run unit tests
# RUN source /workspace/catkin_ws/devel/setup.bash && \
#     cd /workspace/catkin_ws/src/yaml_common && \
#     catkin test --this --no-status
