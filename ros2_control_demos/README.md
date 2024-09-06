docker build . -t rrbot_example -f docker/Dockerfile && docker run -it --rm --name rrbot_example --net host -e ROS_DOMAIN_ID rrbot_example ros2 launch rrbot_example rrbot.launch.py gui:=false

# Must be run quickly after control demo starts
ros2 run joint_state_publisher_gui joint_state_publisher_gui

rviz2 -d rrbot_example/rrbot/rviz/rrbot.rviz

