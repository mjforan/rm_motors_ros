from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    rviz = LaunchConfiguration("rviz")
    rviz_config_file = PathJoinSubstitution([FindPackageShare("gm6020_example"), "config", "gm6020.rviz"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("gm6020_example"), "urdf", "gm6020.urdf.xacro",]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([FindPackageShare("gm6020_example"), "config", "gm6020_controllers.yaml",])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # spawn the default joint controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_position_controller", "--param-file", robot_controllers],
    )

    # spawn alternate controller but leave inactive
    robot_controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "--inactive", "--param-file", robot_controllers],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        robot_controller_spawner_inactive,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
