#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# Nested Launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    follower_gain = LaunchConfiguration('gain')
    follower_gain_launch_arg = DeclareLaunchArgument('gain',default_value='5.0')
    follower_speed = LaunchConfiguration('speed')
    follower_speed_launch_arg = DeclareLaunchArgument('speed',default_value='2.0')

    leader = Node(
        package='turtlesim_control',
        executable='via_point_follower.py',
        namespace="turtle1",
        parameters=[{
            "gain" : 5.0,
            "speed" : 2.0
        },]
    )

    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
        namespace='turtle1'
    )

    # Follower Nested Launch
    turtle_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("turtlesim_control"),
                "launch",
                "turtle_follower_only.launch.py"
            ])
        ]),
        launch_arguments={
            "gain" : follower_gain,
            "speed" : follower_speed
        }.items()
    )

    launch_description = LaunchDescription()

    launch_description.add_action(follower_gain_launch_arg)
    launch_description.add_action(follower_speed_launch_arg)
    launch_description.add_action(leader)
    launch_description.add_action(scheduler)
    # Nested Launch
    launch_description.add_action(turtle_following_launch)
    return launch_description