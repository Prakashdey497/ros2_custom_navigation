import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_config_file_path = os.path.join(get_package_share_directory("turtlebot_waffle_mapping"),'config', 'mapper_params_online_async.yaml')

    
    declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time',
                                                          default_value='true',
                                                          description='Use simulation/Gazebo clock')
    

    declare_slam_params_file_cmd = DeclareLaunchArgument(name='slam_params_file',
                                                         default_value=slam_config_file_path,
                                                         description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(package='slam_toolbox',
                                                  executable='async_slam_toolbox_node',
                                                  name='slam_toolbox',
                                                  output='screen',
                                                  namespace='',
                                                  parameters=[slam_params_file,{'use_sim_time': use_sim_time}])
    

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("turtlebot_waffle_mapping"), "rviz", "maping.rviz")],
    )



    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld