# syntax=docker/dockerfile:1.7-labs
# ^ this can be removed when the `COPY --parents` syntax reaches stable


ARG ROS_DISTRO=jazzy
FROM mjforan/ros-base:${ROS_DISTRO}

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
