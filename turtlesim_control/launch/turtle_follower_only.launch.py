#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gain = LaunchConfiguration('gain')
    gain_launch_arg = DeclareLaunchArgument('gain',default_value='5.0')
    speed = LaunchConfiguration('speed')
    speed_launch_arg = DeclareLaunchArgument('speed',default_value='5.0')

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
        shell=True
    )

    # Goal is Pose of turtle1
    follower = Node(
        package='turtlesim_control',
        executable='turtle_follower.py',
        namespace="turtle2",
        remappings=[
            ('goal', '/turtle1/pose')
        ],
        parameters=[{
            "gain" : gain,
            "speed" : speed
        },]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(turtlesim)
    launch_description.add_action(spawn_turtle2)
    launch_description.add_action(gain_launch_arg)
    launch_description.add_action(speed_launch_arg)
    launch_description.add_action(follower)
    return launch_description