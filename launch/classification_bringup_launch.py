

# import launch
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import (IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# import os

from classifier_importer.classifier_importer import cimporter

def generate_launch_description():

    # Installing Classifiers
    classifiers = []
    classifiers.append({
	    "import_string": "from smap_classifiers.classifier_1 import classifier_1",
	    "class_name": "classifier_1",
	    "args": "\'yolov1\'"
	})
    classifiers.append({
	    "import_string": "from smap_classifiers.classifier_2 import classifier_2",
	    "class_name": "classifier_2",
	    "args": "\'yolov2\'"
	})
    
    cimporter(classifiers)

    # Node Setup

    classification_node = launch_ros.actions.Node(
            package='smap_core',
            executable='classification_node.py',
            name='classification_node',
            output='screen'
    )

    return LaunchDescription([
        classification_node
    ])
