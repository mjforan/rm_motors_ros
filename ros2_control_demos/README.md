TODO
joint limits
multiple joints - how to pass in motor ID?
remove rrbot references and convert to gm6020
add param to toggle simulated operation
tie in to actual gm6020 library
implement controls in gm6020 library


```
docker build . -t rrbot_example -f docker/Dockerfile && docker run -it --rm --name rrbot_example --net host -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION rrbot_example

rviz2 -d rrbot_example/config/rrbot.rviz

# for forward controller
ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{layout:{dim: [], data_offset: 0}, data: [1.0]}" -1

# for joint trajectory controller
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ["joint1"], points: [{positions:[1.0], velocities:[0.0], time_from_start: {sec: 3.0, nanosec: 0.0}}]}" -1
```

To change default controller type, update the `robot_controller_spawner` in rrbot.launch.py

To switch controllers while the system is running:
```
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{activate_controllers: ["forward_effort_controller"], deactivate_controllers: ["joint_trajectory_position_controller"]}'
```