#!/usr/bin/env python3

import rclpy
import traceback
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from semantic_mapping.msg import SmapPrediction

########################################################################
######################## Import the Classifiers ########################
########################################################################

from classification_wrapper.classifiers.classifier_1 import classifier_1
from classification_wrapper.classifiers.classifier_2 import classifier_2

########################################################################
######################## Import the Classifiers ########################
########################################################################

class classification_component(Node):

    classes = []
    parameters = []

    def __init__(self):
        super().__init__("classification_component")

        self.reentrant_cb_group = ReentrantCallbackGroup()

        self.create_subscription(SmapPrediction, '/SMap/classifiers/predictions', self.predict, 10, callback_group=self.reentrant_cb_group)

        self.create_publisher(SmapPrediction, '/SMap/classification_component/predictions', 10, callback_group=self.reentrant_cb_group)
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

    node_list.append(classifier_1())
    #node_list.append(classifier_2())

    ###########################################################
    ######################## Node List ########################
    ###########################################################

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