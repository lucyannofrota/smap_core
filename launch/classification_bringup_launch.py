

# import launch
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import (IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_context import LaunchContext

# import os
import yaml
#from yaml import load
from yaml import CLoader as Loader
from classifier_importer.classifier_importer import cimporter

def generate_launch_description():

    default_config=PathJoinSubstitution([FindPackageShare('smap_core'),'config/classifiers_config.yaml']).perform(LaunchContext())

    # Installing Classifiers
    config_path = ""
    #classifiers.append({
	#    "import_string": "from smap_classifiers.classifier_1 import classifier_1",
	#    "class_name": "classifier_1",
	#    "args": "\'yolov1\'"
	#})
    #classifiers.append({
	#    "import_string": "from smap_classifiers.classifier_2 import classifier_2",
	#    "class_name": "classifier_2",
	#    "args": "\'yolov2\'"
	#})

    ofile = True
    pfile=config_path
    while True:
        try:
            with open(pfile,'r') as file: 
                configs=yaml.load(file)
        except OSError:
            if(ofile):
                ofile = False
                print("Missing input arguments.")
                print("The classifiers shoud be define using a launch file. Example: https://github.com/lucyannofrota/smap_core/blob/master/launch/importer_example_launch.py")
                print("The default config file will be used instead!")
                pfile=default_config
                continue
            else:
                return 1
        break
    
    for cls1 in configs['classifiers']:
        try:
            load_pkg_configs=cls1['external_pkg']
        except:
            load_pkg_configs={'pkg_name': 'smap_classifiers'}
        
        try:
            pkg_path=PathJoinSubstitution([FindPackageShare(load_pkg_configs['pkg_name']),'config/']).perform(LaunchContext())
            print(pkg_path)
            # List files ls
            # Try to found the correct classifier.yaml
            # Open the file and read the configs
            with open(
                PathJoinSubstitution([FindPackageShare(load_pkg_configs['pkg_name']),'config/']).perform(LaunchContext()),
                'r'
            ) as cls1_file:
                configs_cls2=yaml.load(cls1_file)
            dfile=False
            for cls2 in configs_cls2:
                if cls2['name'] == cls1['cname']:
                    dfile=True
                    break
            print("FOUND")
        except:
            print("Fail to load: {}".format(load_pkg_configs['name']))
            pass



    return

    if cimporter(config_path,default_config):
        print("Fail to load config file!")
        return
    #cimporter(classifiers)
    # Node Setup

    classification_node = launch_ros.actions.Node(
            package='smap_core',
            executable='classification_node.py',
            output='screen'
    )

    return LaunchDescription([
        classification_node
    ])

if __name__ == '__main__':
    generate_launch_description()
