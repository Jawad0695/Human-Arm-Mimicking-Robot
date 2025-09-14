import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory("robotic_arm")
    urdf_file = os.path.join(package_share_dir, "urdf", "robotic_arm.urdf")

    return LaunchDescription([
        # Start Gazebo with both required plugins
        ExecuteProcess(
            cmd=[
                "gazebo", "--verbose",
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so"
            ],
            output="screen"
        ),

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[urdf_file],
        ),

        # Spawn robot after 5 seconds delay to ensure Gazebo is ready
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=[
                        "-entity", "robotic_arm",
                        "-file", urdf_file,
                        "-x", "0", "-y", "0", "-z", "0.5"
                    ],
                    output="screen",
                )
            ]
        )
    ])

