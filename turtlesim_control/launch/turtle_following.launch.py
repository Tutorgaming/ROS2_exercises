#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
# Assignment 5 - Execute Process
from launch.actions import ExecuteProcess
# Assignment 6 - Register Event handler
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown

def generate_launch_description():

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    # Assignment 5 - Execute Process
    spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn turtlesim/srv/Spawn \
        "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
        shell=True
    )

    # Assignment 6 - Process Event Handler
    spawn_event = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim,
            on_start=[
                LogInfo(msg="Hello , Turtle Sim is Started ! - Proceed to spawn another turtle"),
                spawn_turtle2
            ]
        )
    )
    sim_close_event = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim,
            on_exit=[
                LogInfo(msg="Arr Shit, Sim is Close"),
                EmitEvent(event=Shutdown(reason='Window closed')),
            ]
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(turtlesim) # Node
    # launch_description.add_action(spawn_turle2) Assignment 6
    # Assignment 6
    launch_description.add_action(spawn_event)
    launch_description.add_action(sim_close_event)

    return launch_description
