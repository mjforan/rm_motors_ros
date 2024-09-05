docker build . -t ros2_control_demos -f Dockerfile/Dockerfile && docker run -it --rm --name ros2_control_demos --net host -e ROS_DOMAIN_ID ros2_control_demos ros2 launch ros2_control_demo_example_1 rrbot.launch.py gui:=false

# Must be run quickly after control demo starts
ros2 run joint_state_publisher_gui joint_state_publisher_gui

rviz2 -d example_1/rrbot/rviz/rrbot.rviz

