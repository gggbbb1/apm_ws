from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from pathlib import Path


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("gazebo_ros")

    # GzServer
    gz_server_launch = (
        Path(pkg_gazebo).joinpath("launch", "gzserver.launch.py").as_posix()
    )
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_server_launch]),
        launch_arguments={
            "pause": "false",
            "verbose": "true",
            "world": "iris.world"
        }.items(),
    )

    #GzClient
    gz_client_launch = (
        Path(pkg_gazebo).joinpath("launch", "gzclient.launch.py").as_posix()
    )
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_client_launch])
    )

    return LaunchDescription(
        [
            gz_server,
            gz_client
        ]
    )