# gm6020_hw

This is a hardware interface used to control DJI GM6020 motors over the CAN bus.

TODO control and feedback interfaces

TODO parameters


## Build System

I originally tried to make the gm6020_can package build natively with colcon but the colcon rust plugins are unmaintained so I couldn't get it working.
Instead it is built in the gm6020_ros CMakeLists.txt using Corrosion. This may seem to stall at 0% in the build process but that is normal.
The output from `cargo build` is buffered and only displayed when the build completes.


## Hardware

Out of the box the motor is configured with ID 0, which is invalid. Make sure to change the DIP switches to your desired ID. TODO CAN termination resistor

This library requires a Linux SocketCAN interface. Example setup using Raspberry Pi 5:

TODO picture

https://www.waveshare.com/wiki/2-CH_CAN_HAT

```
sudo su
raspi-config # enable i2c and SPI
cp 80-can.network /etc/systemd/network/80-can.network
cp 80-can.link /etc/systemd/network/80-can.link
systemctl enable systemd-networkd.service
```

## Simple functionality check

Example from gm6020_can rust package, (mostly) reimplemented in C++

```
colcon build
source install/setup.bash
ros2 run gm6020_hw gm6020_can_test
```


# gm6020_example

Demo package which can drive a gm6020 motor using a position trajectory controller or simple feed-forward controller.


```
docker build . -t gm6020_example -f docker/Dockerfile && docker run -it --rm --name gm6020_example --net host -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION gm6020_example

rviz2 -d gm6020_example/config/gm6020.rviz

# for forward controller
ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{layout:{dim: [], data_offset: 0}, data: [1.0]}" -1

# for joint trajectory controller
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ["joint1"], points: [{positions:[1.0], velocities:[0.0], time_from_start: {sec: 3.0, nanosec: 0.0}}]}" -1
```

To change default controller type, update the `robot_controller_spawner` in gm6020.launch.py

To switch controllers while the system is running:
```
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{activate_controllers: ["forward_effort_controller"], deactivate_controllers: ["joint_trajectory_position_controller"]}'
```


## Software TODO
fix colcon build - unable to find .h or .so files of gm6020_can even though they are present in the install folder. Added
hacky workaround, manual gm6020_can.h in CMakeLists.txt, manual LIBRARY_PATH in Dockerfile.
implement controls in gm6020 library

joint limits - pending updates to ros2_control
