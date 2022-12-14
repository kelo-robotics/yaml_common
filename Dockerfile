FROM ros:melodic

ARG SSH_KEY

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get update -qq && \
    apt-get install -yqq sudo \
                         git \
                         ssh \
                         python-catkin-tools \
                         ros-$ROS_DISTRO-catkin \
                         ros-$ROS_DISTRO-tf \
                         ros-$ROS_DISTRO-tf2-geometry-msgs \
                         libyaml-cpp-dev && \
    apt-get clean

WORKDIR /workspace/catkin_ws/src

# Use SSH deploy keys to clone dependencies and then remove the ssh config
RUN mkdir /root/.ssh/ && \
    echo "$SSH_KEY" > /root/.ssh/id_rsa && \
    chmod -R 600 /root/.ssh/ && \
    touch /root/.ssh/known_hosts && \
    ssh-keyscan -T 60 git.locomotec.com >> /root/.ssh/known_hosts && \
    git clone git@git.locomotec.com:kelo/common/geometry_common.git && \
    rm -rf /root/.ssh/

# Copy the yaml_common source code to the docker container
WORKDIR /workspace/catkin_ws/src/yaml_common
ADD . /workspace/catkin_ws/src/yaml_common/

# Compile the ROS catkin workspace
RUN cd /workspace/catkin_ws && \
    /ros_entrypoint.sh catkin build --limit-status-rate 0.001

# Run unit tests
RUN source /workspace/catkin_ws/devel/setup.bash && \
    cd /workspace/catkin_ws/src/yaml_common && \
    /ros_entrypoint.sh catkin build --this --limit-status-rate 0.001 --catkin-make-args run_tests -- && \
    rosrun yaml_common yaml_common_test
