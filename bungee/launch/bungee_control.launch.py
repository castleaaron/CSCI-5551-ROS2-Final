

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('bungee')
    # bash_script_path = os.path.join(package_dir, 'scripts', 'TerminatorScript.sh')
    
    return LaunchDescription([
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),

        Node(
            package='bungee',
            namespace='bungee',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='bungee',
            namespace='bungee',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        # Node(
        #     package='bungee',
        #     namespace='bungee',
        #     executable='walking_setpoint',
        #     name='walking_setpoint',
        # ),
        # Node(
        #     package='bungee',
        #     namespace='bungee',
        #     executable='control',
        #     name='control',
        #     prefix='gnome-terminal --',
        # ),
        Node(
            package='bungee',
            namespace='bungee',
            executable='xbox_in',
            name='xbox_in',
            prefix='gnome-terminal --',
        ),
        Node(
            package='bungee',
            namespace='bungee',
            executable='velocity_control',
            name='velocity'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])
