

# import launch
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import (IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext

def generate_launch_description():

    node = launch_ros.actions.Node(
            package='smap_core',
            executable='smap_node',
            # name='smap_node',
            namespace='smap',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
    )


    return LaunchDescription([
        node
    ])
