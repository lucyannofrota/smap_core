#!/usr/bin/env python3

import rclpy
import traceback
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from smap_interfaces.msg import SmapPrediction
from smap_interfaces.srv import AddClassifier

# TODO: List of labes service

class classification_component(Node):

    classifiers=[]

    def __init__(self):
        super().__init__("classification")

        self.reentrant_cb_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(SmapPrediction, '/smap_core/classification/classifiers/predictions', self.predict, 10, callback_group=self.reentrant_cb_group)

        self.pub = self.create_publisher(SmapPrediction, '/smap_core/classification/predictions', 10, callback_group=self.reentrant_cb_group)

        self.AddClassifier_srv = self.create_service(AddClassifier, 'add_classifier', self.AddClassifier_callback)
        # Verificar a atomicidade de operações chave

    def AddClassifier_callback(self, request, response):
        self.get_logger().info("Request received.")
        response.is_new=True
        response.success=True
        if(
            (request.name == '') or 
            (request.type == '') or 
            (request.architecture == '') or 
            ( 
                (request.type != 'object') and 
                (request.type != 'place')
            )
        ):
            response.success=False
            self.get_logger().warning("Invalid request received from client.")
            return response

        for cls in self.classifiers:
            if cls['name'] == request.name:
                response.is_new=False
                response.success = (
                    cls['type'] == request.type and 
                    cls['architecture'] == request.architecture
                )
                response.classifier_id=cls['id']
                self.get_logger().warning("Client requesting connection with duplicate classifier name. The id of the first entry will be sent.")
                return response
        if self.classifiers:
            response.classifier_id=self.classifiers[-1]['id']+1
        else:
            response.classifier_id=1
        self.classifiers.append({
            'name': request.name,
            'id': response.classifier_id,
            'type': request.type,
            'architecture': request.architecture
        })
        print(self.classifiers)
        self.get_logger().info("Request successfully processed.")
        return response
    


    def train(self,data):
        pass

    def predict(self,data):
        print(data)

    def on_process(self): # Pooling
        pass


def main(args=None):

    rclpy.init(args=args)

    class_comp = classification_component()

    executor = MultiThreadedExecutor(4)
    executor.add_node(class_comp)

    while(rclpy.ok()):
        try:
            class_comp.on_process()
            executor.spin_once()
        except Exception as exeption:
            traceback_logger_node = Node('node_class_traceback_logger')
            traceback_logger_node.get_logger().error(traceback.format_exc())
            raise exeption

    class_comp.destroy_node()

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()