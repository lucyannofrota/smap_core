#!/usr/bin/env python3

import rclpy
import traceback
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

########################################################################
######################## Import the Classifiers ########################
########################################################################

from classification_wrapper.classifiers.classifier_1 import classifier_1
from classification_wrapper.classifiers.classifier_2 import classifier_2

########################################################################
######################## Import the Classifiers ########################
########################################################################

def main(args=None):

    rclpy.init(args=args)
    node_list = []

    ###########################################################
    ######################## Node List ########################
    ###########################################################

    node_list.append(classifier_1())
    #node_list.append(classifier_2())

    ###########################################################
    ######################## Node List ########################
    ###########################################################

    executor = MultiThreadedExecutor(4)
    for node in node_list:
        executor.add_node(node)

    while(rclpy.ok()):
        try:
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