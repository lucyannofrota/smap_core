

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


    smap_node = launch_ros.actions.Node(
            package='smap_node',
            executable='smap_node',
            # name='smap_node',
            namespace='smap',
            output='screen',
            # parameters=[
            #     {"my_parameter": "earth"}
            # ],
            arguments=['--ros-args', '--log-level', 'info'],
    )

    object_estimator_node = launch_ros.actions.Node(
            package='object_estimator',
            executable='object_estimator_node',
            # name='smap_node',
            namespace='smap',
            output='screen',
            # parameters=[
            #     {"my_parameter": "earth"}
            # ],
            arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        smap_node,
        object_estimator_node
    ])
