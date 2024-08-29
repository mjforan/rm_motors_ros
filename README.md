# GM6020-ROS

This is a hardware interface used to control a single DJI GM6020 motor over the CAN bus. 

TODO control and feedback interfaces

TODO parameters


## Build System

I originally tried to make the gm6020_can package build natively with colcon but the colcon rust plugins are unmaintained so I couldn't get it working.
Instead it is built in the gm6020_ros CMakeLists.txt using Corrosion. This may seem to get stuck at 0% in the build process but that is normal.
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

