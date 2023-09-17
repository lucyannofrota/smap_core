

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

    # rqt = launch.actions.ExecuteProcess(
    #     cmd=['rqt', '--perspective-file', "src/smap/smap_core/config/Debug.perspective"])

    launch_rviz = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d' + "src/smap/smap_core/rviz/smap.rviz"]
    )


    return LaunchDescription([
        # rqt,
        launch_rviz
    ])
