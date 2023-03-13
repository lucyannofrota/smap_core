

# import launch
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import (IncludeLaunchDescription)
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
        cmd=['rqt', '--perspective-file', "src/smap/smap_core/config/Debug.perspective"])

    launch_rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d' + "src/smap/smap_core/config/world.rviz"]
    )

    set_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=True)

    node = launch_ros.actions.Node(
            package='smap_sampler',
            executable='smap_sampler_node',
            name='smap_sampler_node'
    )

    return LaunchDescription([
        rqt,
        launch_file_gazebo,
        launch_nav2,
        launch_slam,
        launch_rviz
        #set_sim_time,
        #node
    ])