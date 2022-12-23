

# import launch
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import (IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
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
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch/navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch/online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    rqt = launch.actions.ExecuteProcess(
        cmd=['rqt', '--perspective-file',
             PathJoinSubstitution([
                    FindPackageShare('semantic_mapping'),
                    'Debug.perspective'
             ])
        ])

    # IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('slam_toolbox'),
    #             'launch/online_async_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': 'true'
    #     }.items()
    # )
    return LaunchDescription([
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         launch_nav2
        #     ]
        # )
        # launch_nav2
        rqt,
        launch_file_gazebo,
        launch_nav2,
        launch_slam
    ])


# launch_ros.actions.Node(
#                     package="rviz2",
#                     executable="rviz2",
#                     name="rviz2",
#                 )
