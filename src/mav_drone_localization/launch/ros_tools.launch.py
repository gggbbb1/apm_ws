import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('mav_drone_localization'), 'config', 'conf.rviz')

    return LaunchDescription([
        # Launch RViz2 with your configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),

        # Launch rqt_gui
        ExecuteProcess(
            cmd=['rqt', '--force-discover'],
            output='screen'
        ),
    ])
