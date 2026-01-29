#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("stella_ahrs"), "config", "params.yaml"
    )

    return LaunchDescription([
        Node(
            package='stella_ahrs',
            executable='stella_ahrs_node',
            name='stella_ahrs_node',
            parameters = [params],
            output='screen'
        )
    ])
