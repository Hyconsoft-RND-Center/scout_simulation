import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch = os.path.join(slam_toolbox, 'launch', 'online_sync_launch.py')

    scout_bot = get_package_share_directory('scout_bot_description')
    params_file = os.path.join(scout_bot, 'config', 'slam_toolbox.yaml')

    use_sim_time = 'true'

    slam_toolbox_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': params_file
        }.items()
    )
    
    return LaunchDescription([
        slam_toolbox_include
    ])