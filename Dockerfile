# syntax=docker/dockerfile:1.7-labs
# ^ this can be removed when the `COPY --exclude` syntax reaches stable

ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-perception
SHELL ["/bin/bash", "-c"]
WORKDIR /colcon_ws
ENTRYPOINT ["/colcon_entrypoint.sh"]
CMD ["bash"]
COPY ./defaults.yaml /root/.colcon/defaults.yaml
COPY ./colcon_entrypoint.sh /colcon_entrypoint.sh

ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update && \
    rosdep update 

RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Rust
ENV PATH=/root/.cargo/bin:$PATH
RUN apt install -y git libclang-dev python3-pip curl && \
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    cargo install cargo-ament-build && \
    pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git

RUN apt install -y nano

COPY --exclude=./*/src/ ./* ./src/
RUN rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

COPY ./* ./src/
RUN . /ros_entrypoint.sh && \
    colcon build
