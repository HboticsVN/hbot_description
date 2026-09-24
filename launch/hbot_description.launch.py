"""Deprecated: kept so existing commands keep working. Use instead

  description.launch.py   robot_state_publisher only (include it from bringup)
  view.launch.py          + joint_state_publisher + RViz

This wrapper maps the old `sim` argument to `use_sim` and runs view.launch.py
(RViz off by default, as before).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
  pkg_share = get_package_share_directory('hbot_description')
  return LaunchDescription([
    DeclareLaunchArgument('sim', default_value='false',
                          description='Deprecated alias of use_sim'),
    DeclareLaunchArgument('rviz', default_value='false', description='Open RViz'),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'view.launch.py')),
      launch_arguments={
        'use_sim': LaunchConfiguration('sim'),
        'rviz': LaunchConfiguration('rviz'),
      }.items()),
  ])
