#!/usr/bin/env python3

import rclpy
import traceback
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from smap_interfaces.msg import SmapPrediction

########################################################################
######################## Import the Classifiers ########################
########################################################################

### <import_classifiers> ###
### </import_classifiers> ###

########################################################################
######################## Import the Classifiers ########################
########################################################################

class classification_component(Node):

    classes = []
    parameters = []

    def __init__(self):
        super().__init__("classification")

        self.reentrant_cb_group = ReentrantCallbackGroup()

        self.create_subscription(SmapPrediction, '/smap_core/classification/classifiers/predictions', self.predict, 10, callback_group=self.reentrant_cb_group)

        self.create_publisher(SmapPrediction, '/smap_core/classification/predictions', 10, callback_group=self.reentrant_cb_group)
        # Verificar a atomicidade de operações chave


    def train(self,data):
        pass

    def predict(self,data):
        print(data)

    def on_process(self): # Pooling
        pass


def main(args=None):

    rclpy.init(args=args)

    class_comp = classification_component()
    node_list = []

    ###########################################################
    ######################## Node List ########################
    ###########################################################

    ### <append_classifiers> ###
    ### </append_classifiers> ###

    ###########################################################
    ######################## Node List ########################
    ###########################################################

    if not node_list:
        class_comp.get_logger().fatal("No classifiers detected!\nThe classifiers shoud be define using a launch file. Example: https://github.com/lucyannofrota/smap_core/blob/master/launch/importer_example_launch.py")
        rclpy.shutdown()
        return 1

    executor = MultiThreadedExecutor(4)
    executor.add_node(class_comp)
    for node in node_list:
        executor.add_node(node)

    while(rclpy.ok()):
        try:
            class_comp.on_process()
            for node in node_list:
                node.on_process() # Node Pooling
            executor.spin_once()
        except Exception as exeption:
            traceback_logger_node = Node('node_class_traceback_logger')
            traceback_logger_node.get_logger().error(traceback.format_exc())
            raise exeption

    for node in node_list:
        node.destroy_node()

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()