# syntax=docker/dockerfile:1.7-labs
# ^ this can be removed when the `COPY --exclude` syntax reaches stable

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]
WORKDIR /colcon_ws
ENTRYPOINT ["/colcon_entrypoint.sh"]
CMD ros2 launch gm6020_example gm6020.launch.py gui:=false

COPY ./docker/colcon-defaults.yaml /root/.colcon/defaults.yaml
COPY ./docker/colcon_entrypoint.sh /colcon_entrypoint.sh

RUN apt update && apt upgrade -y && apt install -y \
    nano curl ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install Rust
ENV PATH=/root/.cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

COPY --parents ./**/package.xml /colcon_ws/src/
RUN rosdep update --rosdistro ${ROS_DISTRO} && apt update && \
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
RUN apt-get autoremove -y -qq && rm -rf /var/lib/apt/lists/*

COPY ./gm6020_can ./src/gm6020_can
RUN . /ros_entrypoint.sh && \
    colcon build --packages-select gm6020_can

COPY ./gm6020_hw ./src/gm6020_hw
RUN . /ros_entrypoint.sh && \
    LIBRARY_PATH=/colcon_ws/install/gm6020_can/lib:$LIBRARY_PATH colcon build --packages-select gm6020_hw

COPY ./gm6020_example ./src/gm6020_example
RUN . /ros_entrypoint.sh && \
    colcon build --packages-select gm6020_example

#COPY . ./src
#RUN . /ros_entrypoint.sh && \
#    colcon build