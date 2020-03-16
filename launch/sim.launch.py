import launch
from launch import LaunchDescription

import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rj-robocup', node_executable='grsim',
            arguments=['--headless']),
        launch_ros.actions.Node(
            package='rj-robocup', node_executable='soccer',
            output='screen',
            arguments=['-y', '-sim'],
            on_exit=launch.actions.Shutdown()),
    ])
