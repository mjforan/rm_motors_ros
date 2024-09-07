```
docker build . -t rrbot_example -f docker/Dockerfile && docker run -it --rm --name rrbot_example --net host -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION rrbot_example ros2 launch rrbot_example rrbot.launch.py gui:=false

rviz2 -d rrbot_example/config/rrbot.rviz

# for forward controller
ros2 topic pub /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{layout:{dim: [], data_offset: 0}, data: [1.0]}" -1

# for joint trajectory controller
ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory "{joint_names: ["joint1"], points: [{positions:[2.0], velocities:[0.0], time_from_start: {sec: 3.0, nanosec: 0.0}}]}" -1
```

To change controller type, update the `robot_controller_spawner` in rrbot.launch.py

To switch while the system is running (inconsistent):
```
ros2 run controller_manager spawner forward_position_controller --inactive
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```