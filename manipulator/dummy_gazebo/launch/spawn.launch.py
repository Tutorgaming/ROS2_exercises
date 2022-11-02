#!usr/bin/python3

"""
This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

import os, yaml
import xacro
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from dummy_description.DH2Transform import DH2Transform
class ShareFile():
    def __init__(self,package,folder,file):
        self.package_name = package
        self.package_path = get_package_share_directory(package)
        self.folder = folder
        self.file = file
        self.path = os.path.join(self.package_path,folder,file)
def launch_action_gazebo():
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ])
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )
    return gazebo_server,gazebo_client
def launch_action_robot_spawner(dh_parameters, robot_description, controller, position, robot_name=""):
    # robot_state_publisher
    DH2Transform(dh_parameters.package_name,dh_parameters.folder,dh_parameters.file)
    parameters = []
    #############################################
    # TODO 6: Many Robot
    #############################################
    if robot_name:
        robot_desc_xml = xacro.process_file(
            robot_description.path,
            mappings={'robot_name':robot_name}
    ).toxml()
    else:
        robot_desc_xml = xacro.process_file(
            robot_description.path
        ).toxml()
    parameters.append({
        'robot_description':robot_desc_xml,
        "frame_prefix": str(robot_name) + "/" if robot_name else ""   # TODO 6
    })
    parameters.append({'use_sim_time': True})

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=parameters,
        namespace=robot_name,  # TODO 6
    )
    #############################################
    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic',  str(robot_name) + '/robot_description',
            '-entity', str(robot_name) + "/dummy", # TODO 6
            '-x',str(position[0]),
            '-y',str(position[1]),
            '-z',str(position[2]),
            '-R','0',
            '-P','0',
            '-Y','0',
        ]
    )

    # Create Controller for those joints of robot
    # Commander (to Driver)
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_position_controller",
            "--controller-manager", "{}/controller_manager".format(robot_name),
        ],
    )
    # Interface (Driver) Reader
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "{}/controller_manager".format(robot_name),
        ]
    )
    # Trajectory Generator (High Level Goal Gen for control test)
    traj_gen = Node(
        package="dummy_control",
        executable="trajectory_generator.py",
        namespace=robot_name
    )
    actions = []
    actions.append(robot_state_publisher)
    actions.append(spawner)
    actions.append(joint_trajectory_controller)
    actions.append(joint_state_broadcaster)
    actions.append(traj_gen)
    return actions

def generate_controller_config(controller:ShareFile, namespace):
    #############################################
    # TODO 6: Spawn Many Robot
    #############################################
    # Code Here
    # Open Default Config and nested under the NAMESPACE for ease of on-the-fly yaml generate for controller launch
    with open(controller.path, "r") as file:
        doc = yaml.load(file, yaml.SafeLoader)
        new_doc = {
            namespace : doc
        }
        controller_config_path = os.path.join(controller.package_path, controller.folder, namespace+controller.file)
        with open(controller_config_path, "w") as new_file:
            yaml.dump(new_doc, new_file)
    return controller_config_path
    #############################################

def generate_launch_description():
    dh_parameters = ShareFile('dummy_description','config','DH_parameters.yaml')
    robot_description = ShareFile('dummy_gazebo','robot','dummy.xacro')
    # Code Here
    controller = ShareFile('dummy_gazebo','config','_controller_config.yaml')
    gazebo_server,gazebo_client = launch_action_gazebo()
    #############################################
    # TODO 6: Spawn Many Robot
    #############################################
    actions = []
    # robot_name = "my_robot_1"
    for i in range(0,5):
        robot_name = "dummy_{}".format(i)
        new_controller_yaml_config = generate_controller_config(controller, robot_name)
        actions += launch_action_robot_spawner(
            dh_parameters,
            robot_description,
            new_controller_yaml_config,
            [0.0,i * 2.0,0.0],
            robot_name,
        )
    #############################################
    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)
    for action in actions:
        launch_description.add_action(action)

    return launch_description


    """
    # EXEC

    ros2 service call /dummy_0/set_joints dummy_kinematics_interfaces/srv/SetConfig "joint_states:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  name: []
  position: []
  velocity: []
  effort: []" ; ros2 service call /dummy_1/set_joints dummy_kinematics_interfaces/srv/SetConfig "joint_states:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  name: []
  position: []
  velocity: []
  effort: []" ; ros2 service call /dummy_2/set_joints dummy_kinematics_interfaces/srv/SetConfig "joint_states:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  name: []
  position: []
  velocity: []
  effort: []" ; ros2 service call /dummy_3/set_joints dummy_kinematics_interfaces/srv/SetConfig "joint_states:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  name: []
  position: []
  velocity: []
  effort: []" ; ros2 service call /dummy_4/set_joints dummy_kinematics_interfaces/srv/SetConfig "joint_states:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  name: []
  position: []
  velocity: []
  effort: []"


    """