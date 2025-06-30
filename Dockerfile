FROM ros:jazzy

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get -qq update > /dev/null && \
    apt-get -yqq install git > /dev/null

# Clone dependencies
WORKDIR /workspace/ros2_ws/src
RUN git clone -b ros2 https://github.com/kelo-robotics/geometry_common.git

# Copy the yaml_common source code to the docker container
WORKDIR /workspace/ros2_ws/src/yaml_common
COPY . .

# Build
WORKDIR /workspace/ros2_ws
RUN /ros_entrypoint.sh colcon build

# Test
RUN . install/setup.bash && colcon test --packages-up-to yaml_common --event-handlers console_direct+
RUN mkdir /test_results
RUN cp /workspace/ros2_ws/build/yaml_common/test_results/yaml_common/yaml_common_test.gtest.xml /test_results/
