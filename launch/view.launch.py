"""Show the HBOT model in RViz, without the robot or Gazebo.

  ros2 launch hbot_description view.launch.py                 # real-robot model
  ros2 launch hbot_description view.launch.py use_sim:=true   # Gazebo model

With use_sim:=true the wheels are continuous joints, so a joint_state_publisher
is started to give them a state (gui:=true: sliders, needs
joint_state_publisher_gui).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
  pkg_share = get_package_share_directory('hbot_description')

  use_sim = LaunchConfiguration('use_sim')
  joint_state_publisher = LaunchConfiguration('joint_state_publisher')
  gui = LaunchConfiguration('gui')

  description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_share, 'launch', 'description.launch.py')),
    launch_arguments={
      'use_sim': use_sim,
      # No Gazebo here, so there is no /clock to follow.
      'use_sim_time': 'false',
      'driver_joint_states': LaunchConfiguration('driver_joint_states'),
    }.items())

  return LaunchDescription([
    DeclareLaunchArgument('use_sim', default_value='false',
                          description='false: real robot; true: Gazebo model'),
    DeclareLaunchArgument('driver_joint_states', default_value='false',
                          description='Make the real wheels continuous joints'),
    DeclareLaunchArgument('joint_state_publisher', default_value=use_sim,
                          description='Publish wheel joint states (defaults to use_sim)'),
    DeclareLaunchArgument('gui', default_value='false',
                          description='Use joint_state_publisher_gui sliders'),
    DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
    DeclareLaunchArgument('rvizconfig',
                          default_value=os.path.join(pkg_share, 'rviz', 'hbot.rviz'),
                          description='Absolute path to the RViz config'),
    description,
    Node(
      package='joint_state_publisher',
      executable='joint_state_publisher',
      condition=IfCondition(PythonExpression(
        ["'", joint_state_publisher, "' == 'true' and '", gui, "' != 'true'"]))),
    Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      condition=IfCondition(gui)),
    Node(
      package='rviz2',
      executable='rviz2',
      output='screen',
      arguments=['-d', LaunchConfiguration('rvizconfig')],
      condition=IfCondition(LaunchConfiguration('rviz'))),
  ])
