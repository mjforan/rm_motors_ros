# syntax=docker/dockerfile:1.7-labs
# ^ this can be removed when the `COPY --parents` syntax reaches stable

# Start from the ROS 2 base image
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Set up some container details
# Avoid waiting for user input when installing packages
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /colcon_ws
ENTRYPOINT ["/colcon_entrypoint.sh"]
CMD ros2 launch rm_motors_example rm_motors.launch.py gui:=false

COPY ./docker/colcon-defaults.yaml /root/.colcon/defaults.yaml
COPY ./docker/colcon_entrypoint.sh /colcon_entrypoint.sh

# Install system packages
RUN apt update && apt upgrade -y && apt install -y \
    nano curl ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install Rust
ENV PATH=/root/.cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN cargo install cargo-expand

# Copy ONLY the package.xml files but put them in the correct directory structure.
# This allows us to install ROS dependencies separately and save time when rebuilding just the code.
COPY --parents ./**/package.xml /colcon_ws/src/
RUN rosdep update --rosdistro ${ROS_DISTRO} && apt update && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
RUN apt-get autoremove -y -qq && rm -rf /var/lib/apt/lists/*

# Copy the code and build
COPY ./rm_motors_hw ./src/rm_motors_hw
COPY ./rm_motors_example ./src/rm_motors_example
RUN . /ros_entrypoint.sh && \
    colcon build
