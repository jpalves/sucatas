from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
             package='sucatas_hw', executable='ticks.py', output='screen'),
        launch_ros.actions.Node(
             package='sucatas_hw', executable='diff_tf.py', output='screen')
    ])
