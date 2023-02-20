#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from classifiers.classifier_1 import classifier_1
from classifiers.classifier_2 import classifier_2

from rclpy.node import Node

import traceback

def main(args=None):

    rclpy.init(args=args)

    ###########################################################
    ######################## Node List ########################
    ###########################################################

    node_list = []
    node_list.append(classifier_1())
    node_list.append(classifier_2())

    ###########################################################
    ######################## Node List ########################
    ###########################################################

    executor = MultiThreadedExecutor(4)
    for node in node_list:
        executor.add_node(node)

    while(rclpy.ok()):
        try:
            for node in node_list:
                node.on_process() # Pooling
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