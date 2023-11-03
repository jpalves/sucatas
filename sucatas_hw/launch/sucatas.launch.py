import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.actions

lidar = get_package_share_directory('rplidar_ros')

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
             package='sucatas_hw', executable='ticks.py', output='screen'),
        launch_ros.actions.Node(
             package='sucatas_hw', executable='diff_tf.py', output='screen'),
        IncludeLaunchDescription(
                      PythonLaunchDescriptionSource(os.path.join(lidar, 'launch','rplidar.launch.py')),
                      launch_arguments={
                              "namespace": "",
                              "use_namespace": "False",
                       }.items(),
         )
    ])
