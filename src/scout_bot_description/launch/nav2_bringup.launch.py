from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='scout_bot_description').find('scout_bot_description')

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'nav2_param.yaml'),
                        {'use_sim_time': True}]
        )
    ])