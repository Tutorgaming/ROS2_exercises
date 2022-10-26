#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
# Assignment 4 - Launch Arguments
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""
Assignment 1
- Use the Remapping to have the namespace turtle1 in front of every topic
"""

def generate_launch_description():

    # Assignment 4 - Launch Arguments
    gain = LaunchConfiguration("gain")
    gain_launch_arg = DeclareLaunchArgument("gain", default_value="5.0")

    speed = LaunchConfiguration("speed")
    speed_launch_arg = DeclareLaunchArgument("speed", default_value="3.0")

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    controller = Node(
        package='turtlesim_control',
        executable='controller.py',
        namespace="turtle1",
        # Assignment 3 - Insert the Parameters
        parameters=[
            {'gain': gain},
            {'speed': speed},
        ]
    )
    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
        namespace="turtle1",
    )
    # Add Action  -> Input Node
    launch_description = LaunchDescription()
    launch_description.add_action(gain_launch_arg) # DeclareLaunchArgument
    launch_description.add_action(speed_launch_arg) # DeclareLaunchArgument
    launch_description.add_action(turtlesim) # Node
    launch_description.add_action(controller) # Node
    launch_description.add_action(scheduler) # Node
    return launch_description
