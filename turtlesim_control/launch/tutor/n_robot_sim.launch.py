#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def spawn_n_robot(count):
    result = []
    for i in range(1,count):
        command = "ros2 service call /spawn turtlesim/srv/Spawn"
        detail = "{x: 2.0, y: 2.0, theta: 0.0, name:\'turtle%d\' }" % (i)
        print(command + detail)
    return result
spawn_n_robot(5)


def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[
            {'background_b':100},
            {'background_g':20},
            {'background_r':new_background_r},
        ]
    )
