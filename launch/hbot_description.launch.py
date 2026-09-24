import launch
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='hbot_description').find('hbot_description')
  default_model_path = os.path.join(pkg_share, 'urdf', 'hbot.urdf.xacro')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'hbot.rviz')

  print('default_model_path : {}'.format(default_model_path))

  robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
        'robot_description': ParameterValue(
            Command(['xacro', ' ', LaunchConfiguration('model'),
                     ' sim:=', LaunchConfiguration('sim')]),
            value_type=str)}]
  )

  rviz_node = launch_ros.actions.Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')],
    condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
  )
  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                          description='Absolute path to rviz config file'),
    launch.actions.DeclareLaunchArgument(name='rviz', default_value='false',
                                          description='Open RViz?'),
    launch.actions.DeclareLaunchArgument(name='sim', default_value='false',
                                          description='false: real-robot frames only (hbot.urdf); '
                                                      'true: full Gazebo model (hbot_sim.urdf)'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    robot_state_publisher_node,
    rviz_node
  ])
