import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bot_description = get_package_share_directory("bot_description")
    bot_description_prefix = get_package_prefix("bot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")


    world = os.path.join(get_package_share_directory('bot_world'),'world','world01.world')

    # Set the environment variable
    model_path = os.path.join(bot_description, "models")
    model_path += pathsep + os.path.join(bot_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)


    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time',
                                                     default_value='true',
                                                     description='Use simulation (Gazebo) clock if true')


    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bot_description,'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={'world': world}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot_node = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "bumperbot",
                                   "-topic", "robot_description",
                                   "-x", "0.0",
                                   "-y", "0.0", 
                                   '-z', '0.01'      
                                  ],
                        output="screen"
    )


    ld = LaunchDescription()

    ld.add_action(env_var)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)
    ld.add_action(spawn_robot_node)

    return ld