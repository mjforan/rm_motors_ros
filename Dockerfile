# syntax=docker/dockerfile:1.7-labs
# ^ this can be removed when the `COPY --exclude` syntax reaches stable

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
SHELL ["/bin/bash", "-c"]
WORKDIR /colcon_ws
ENTRYPOINT ["/colcon_entrypoint.sh"]
CMD ["bash"]
COPY ./colcon-defaults.yaml /root/.colcon/defaults.yaml
COPY ./colcon_entrypoint.sh /colcon_entrypoint.sh

ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update && apt install -y nano curl ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Rust
ENV PATH=/root/.cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

COPY --parents ./*/package.xml ./src/
RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

COPY ./ ./src/
RUN . /ros_entrypoint.sh && \
    colcon build
