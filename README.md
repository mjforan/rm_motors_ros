# gm6020_hw

This is a hardware interface wrapping the [`gm6020_can`](https://github.com/mjforan/gm6020_can) library, used to control DJI GM6020 motors over the CAN bus. It exposes `velocity` and `effort` command interfaces, with `position`, `velocity`, `effort`, and `temperature` state interfaces. Top level parameters are `can_interface` (e.g. "can0") and the boolean `simulate` for testing without a motor connected. Each joint must provide a `gm6020_id` parameter so the hardware interface knows which motor to use.

For more information on ROS 2 Control hardware interfaces, check the [documentation](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html#) and [examples](https://github.com/ros-controls/ros2_control_demos).


# gm6020_example

This is a demonstration package which can drive gm6020 motors using a position trajectory controller or simple feed-forward controller.

Big thanks to the authors of [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) for providing helpful starter code.


# Important Files

### [gm6020_controllers.yaml](gm6020_example/config/gm6020_controllers.yaml)
Controller details: joint names, command/state interfaces, PID gains.

### [gm6020.launch.py](gm6020_example/launch/gm6020.launch.py)
Launch `controller_manager`, `robot_state_publisher`, `joint_state_broadcaster`, spawn the controllers, and (optionally) start RViz.

### [gm6020_description.urdf.xacro](gm6020_example/urdf/gm6020_description.urdf.xacro)
Physical configuration of the joints: pose, size, color, intertia, etc.

### [gm6020.ros2_control.xacro](gm6020_example/urdf/gm6020.ros2_control.xacro)
Hardware interface description: command/state interfaces, limits (currently non-functional), parameters.


# Build System

The `gm6020_can` Rust library is built in the `gm6020_ros` CMakeLists.txt. [Corrosion](https://corrosion-rs.github.io/corrosion/) compiles the crate and creates a library target for `target_link_library`. The build process may seem to stall at 0% but that is normal- the output from `cargo build` is buffered and only displayed when the build completes.


# Hardware Setup

Out of the box the motor is configured with ID 0, which is invalid. Use the DIP switches to set the ID and enable the CAN termination resistor if necessary.

This library requires a Linux SocketCAN interface. It was tested using a Raspberry Pi 5 and the [Waveshare CAN HAT](https://www.waveshare.com/wiki/2-CH_CAN_HAT). Quick start setup:

```
sudo su
raspi-config # enable i2c and SPI
cp 80-can.network /etc/systemd/network/80-can.network
cp 80-can.link /etc/systemd/network/80-can.link
systemctl enable systemd-networkd.service

echo """
dtparam=spi=on
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=25
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
dtoverlay=spi-bcm2835-overlay
""">> /boot/firmware/config.txt
reboot now
```

For debugging, you may want to `sudo apt install can-utils`



# Software Setup
Must have ROS 2 and Rust installed, with `gm6020_hw` and `gm6020_example` in a colcon workspace. The Docker image (see [below](#docker)) will do this for you.


# Simple Functionality Check

Example from `gm6020_can` Rust package (mostly) reimplemented in C++.
```
colcon build
. install/setup.bash
ros2 run gm6020_hw gm6020_can_test
```


# Full Stack Demo

The hardware interface uses the low-level library to communicate to the motor. The trajectory controller commands the hardware interface to move through an arbitrary sequence of positions and velocities. RViz2 visualizes the system.

```
ros2 launch gm6020_example gm6020.launch.py

# For joint trajectory controller
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ["joint1"], points: [{positions:[1.0], velocities:[0.0], time_from_start: {sec: 3.0, nanosec: 0.0}}]}" -1

# To switch controllers while the system is running:
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{deactivate_controllers: ["joint_trajectory_position_controller"], activate_controllers: ["forward_effort_controller"], strictness: 2}'

# For feed-forward controller
ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{layout:{dim: [], data_offset: 0}, data: [1.0]}" -1
```


# Software TODO
- [ ] multithreading in gm6020_hw.cpp
- [ ] test on hardware
- [ ] joint limits
   - [ ] pending updates to ros2_control
   - [ ] dynamic reconfigure?


# Docker

A [Docker configuration](docker/Dockerfile) is provided for ease of setup and deployment.

```
git clone https://github.com/mjforan/gm6020_ros.git
cd gm6020_ros
git submodule update --init
docker build . -t mjforan/gm6020-ros -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy
docker run --rm -it --name gm6020_ros --network host -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION mjforan/gm6020-ros
```