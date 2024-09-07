# syntax=docker/dockerfile:1.7-labs

ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

COPY --parents ./**/package.xml /rrbot_ws/src/
WORKDIR /rrbot_ws

RUN rosdep update --rosdistro ${ROS_DISTRO}  \
    && apt-get update \
    && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2\
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

COPY . /rrbot_ws/src

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
WORKDIR /rrbot_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ros2 launch rrbot_example rrbot.launch.py start_rviz:=false
