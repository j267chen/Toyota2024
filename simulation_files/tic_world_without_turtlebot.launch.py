import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

  world = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'),
    'worlds',
    'tic_field_v1.world'
  )
  # Declare the launch argument for the world file
  declare_world_cmd = DeclareLaunchArgument(
      'world',
      default_value=world,
      description='Full path to the world file to load'
  )

  # Use LaunchConfiguration to get the world file path
  world = LaunchConfiguration('world')

  return LaunchDescription([
	
	declare_world_cmd,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
            output='screen')
  ])

