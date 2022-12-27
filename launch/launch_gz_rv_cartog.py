

# import launch
from launch import LaunchDescription
# import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# import os


def generate_launch_description():
    launch_file_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch/turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    launch_file_rviz_carto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),
                'launch/cartographer.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    return LaunchDescription([
        launch_file_gazebo,
        launch_file_rviz_carto
    ])
