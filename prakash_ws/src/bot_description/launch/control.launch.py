import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    bot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bot_controller",
                "--controller-manager",
                "/controller_manager"
        ]
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            bot_controller,
        ]
    )

            