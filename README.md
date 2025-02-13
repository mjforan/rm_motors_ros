[Quick Start](#quick-start)

# rm_motors_hw

This is a `ros2_control` hardware interface wrapping the [`rm_motors_can`](https://github.com/mjforan/rm_motors_can) library, used to control DJI RoboMaster motors over the CAN bus.


<table>
<tr><td>

| Command Interface |
|--------------------|
| `velocity` |
| `effort`   |

</td><td></td><td></td><td>

| State Interface |
|------------------|
| `position`    |
| `velocity`    |
| `effort`      |
| `temperature` |

</td></tr></table>

Only one command interface may be designated for each motor. The interface type is linked to the motor mode {voltage: velocity, current: effort}.

| Parameter | Required | Type | Description |
|-----------|----------|------|-------------|
| `can_interface`         | true  | string  | Network interface to use e.g. "can0"    |
| `simulate`              | false | boolean | For testing without a motor connected   |
| `joint/motor_id`        | true  | integer | Which motor to use for this joint       |
| `joint/motor_type`      | true  | string  | {"gm6020", "m3508", "m2006"}            |
| `joint/position_offset` | false | double  | Set the "zero" position of the motor    |

These parameters are set in [rm_motors.ros2_control.xacro](rm_motors_example/urdf/rm_motors.ros2_control.xacro), not as regular ROS node parameters.

For more information on ROS 2 Control hardware interfaces, check the [documentation](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html#) and [examples](https://github.com/ros-controls/ros2_control_demos).


# rm_motors_example

This is a demonstration package which drives a gm6020 motor using a position trajectory controller or simple feed-forward controller.

Thanks to the authors of [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) for helpful starter code.


# Important Files

### [rm_motors_controllers.yaml](rm_motors_example/config/rm_motors_controllers.yaml)
Controller details: joint names, command/state interfaces, PID gains.

### [rm_motors.launch.py](rm_motors_example/launch/rm_motors.launch.py)
Launch `controller_manager`, `robot_state_publisher`, `joint_state_broadcaster`, spawn the controllers, and (optionally) start RViz.

### [rm_motors_description.urdf.xacro](rm_motors_example/urdf/rm_motors_description.urdf.xacro)
Physical configuration of the joints: pose, size, color, intertia, etc.

### [rm_motors.ros2_control.xacro](rm_motors_example/urdf/rm_motors.ros2_control.xacro)
Hardware interface description: command/state interfaces, limits (currently non-functional), parameters.


# Build System

The `rm_motors_can` Rust library is built in the `rm_motors_ros` CMakeLists.txt. [Corrosion](https://corrosion-rs.github.io/corrosion/) compiles the crate and creates a library target for `target_link_library`. The build process may seem to stall at 0% but that is normal- the output from `cargo build` is buffered and only displayed when the build completes. TODO for unknown reasons the `rm_motors_can` build takes a long time to complete.


# Hardware Setup

Out of the box the motor is configured with ID 0, which is invalid. Beware that GM6020 ID 1-4 cannot coexist with M3508/M2006 ID 5-8 due to overlapping CAN Bus addresses. Use the DIP switches to set the ID and enable the CAN termination resistor if necessary.

This library requires a Linux SocketCAN interface. It was tested using a Raspberry Pi 5 and the [Waveshare CAN HAT](https://www.waveshare.com/wiki/2-CH_CAN_HAT). Quick start setup:

```
sudo su
raspi-config # enable SPI
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
TODO `NetworkManager` is installed by default but it does not support SocketCAN interfaces and it should not coexist with `systemd-networkd`.

For debugging, you may want to `sudo apt install -y can-utils`, which adds useful commands such as `candump` and `cansend`.

Connect the red wire to the "H" pin on CAN0, with the black wire going to the "L" pin. If these are the only two hosts on the CAN bus, set the HAT jumper and motor DIP switch to enable the CAN termination resistors. Power the motor with 24VDC 4A.

Beware that the motor reaches temperatures which can soften common 3d-printed materials.


# Software Setup
Must have ROS 2 and Rust installed, with `rm_motors_hw` and `rm_motors_example` in a colcon workspace. Don't forget to install dependencies with rosdep, and `cargo install cargo-expand`. The [Docker image](#docker) will do all the setup for you.

The rm_motors_can library has some [simple examples]([rm_motors_hw/rm_motors_can/README.md](https://github.com/mjforan/rm_motors_can/blob/main/README.md#c-example) to try.


# Full Stack Demo

[![Demo Video](https://img.youtube.com/vi/UEskAxSjQE4/0.jpg)](https://www.youtube.com/watch?v=UEskAxSjQE4)

The hardware interface uses the low-level library to communicate to the motor. The trajectory controller commands the hardware interface to move through an arbitrary sequence of positions and velocities. RViz2 visualizes the system.

```
. install/setup.bash

ros2 launch rm_motors_example rm_motors.launch.py &  # don't forget to kill this when you're done

# For joint trajectory controller
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ["joint1"], points: [{positions:[0.0], velocities:[0.0], time_from_start: {sec: 3, nanosec: 0}}]}" -1

# To switch controllers while the system is running:
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{deactivate_controllers: ["joint_trajectory_position_controller"], activate_controllers: ["forward_effort_controller"], strictness: 2}'

# For feed-forward controller
ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{layout:{dim: [], data_offset: 0}, data: [1.0]}" -1
```


# Docker

A [Docker configuration](Dockerfile) is provided for ease of setup and deployment. There is also a dev container configuration for use with Visual Studio Code.

```
docker build . -t mjforan/rm-motors-ros:jazzy --build-arg ROS_DISTRO=jazzy
docker run --rm -it --name rm_motors_ros --network host -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION mjforan/rm-motors-ros:jazzy
```


# Quick Start

Configure your hardware so a motor is available on a Linux CAN network interface. [Example Setup](#hardware-setup)

```
git clone https://github.com/mjforan/rm_motors_ros.git
cd rm_motors_ros
git submodule update --init
```

Edit parameters in [rm_motors.ros2_control.xacro](rm_motors_example/urdf/rm_motors.ros2_control.xacro)

[Install Docker](https://docs.docker.com/engine/install/)

```
docker build . -t mjforan/rm-motors-ros -f Dockerfile --build-arg ROS_DISTRO=jazzy
docker run --rm -it --name rm_motors_ros --network host -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION mjforan/rm-motors-ros
```

Open a new terminal or use `docker run -d ...` to put it in the background.

```
docker exec -it rm_motors_ros bash
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ["joint1"], points: [{positions:[0.0], velocities:[0.0], time_from_start: {sec: 3, nanosec: 0}}]}" -1
```

Change the position or add points to move the motor. Be cautious as the default PID values may not work for your system.

Additional steps are necessary to make RViz visible from the container. I recommend Visual Studio Code with the Docker extension.
