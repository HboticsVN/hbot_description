"""robot_state_publisher for the HBOT, with the description expanded at launch.

This is the launch file other packages should include to get /robot_description
and the static TF of the robot:

  IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(
          get_package_share_directory('hbot_description'),
          'launch', 'description.launch.py')),
      launch_arguments={'use_sim': 'true', 'use_sim_time': 'true'}.items())

Arguments:
  use_sim              false: real robot, true: + Gazebo sensors/plugins
  use_sim_time         defaults to use_sim
  driver_joint_states  true once hbot_driver publishes /joint_states
                       (makes the wheels continuous joints on the real robot)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
  default_model = os.path.join(
    get_package_share_directory('hbot_description'), 'urdf', 'hbot.urdf.xacro')

  model = LaunchConfiguration('model')
  use_sim = LaunchConfiguration('use_sim')
  use_sim_time = LaunchConfiguration('use_sim_time')
  driver_joint_states = LaunchConfiguration('driver_joint_states')

  # ParameterValue(..., value_type=str): without it, Humble's launch tries to
  # parse the xacro output as YAML and fails.
  robot_description = ParameterValue(
    Command(['xacro ', model,
             ' use_sim:=', use_sim,
             ' driver_joint_states:=', driver_joint_states]),
    value_type=str)

  return LaunchDescription([
    DeclareLaunchArgument('model', default_value=default_model,
                          description='Absolute path to the robot xacro'),
    DeclareLaunchArgument('use_sim', default_value='false',
                          description='false: real robot; true: Gazebo model'),
    DeclareLaunchArgument('use_sim_time', default_value=use_sim,
                          description='Use the Gazebo /clock (defaults to use_sim)'),
    DeclareLaunchArgument('driver_joint_states', default_value='false',
                          description='true once the driver publishes /joint_states'),
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{'use_sim_time': use_sim_time,
                   'robot_description': robot_description}]),
  ])
