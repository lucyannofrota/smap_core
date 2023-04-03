#!/usr/bin/env python3

import rclpy
import traceback
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from smap_interfaces.msg import SmapPrediction
from smap_interfaces.srv import AddPerceptionModule

class classification_component(Node):

    detectors=[]
    classes=[]

    def __init__(self):
        super().__init__("classification")

        self.reentrant_cb_group = ReentrantCallbackGroup()

        #self.sub = self.create_subscription(SmapPrediction, '/smap_core/perception/detectors/predictions', self.predict, 10, callback_group=self.reentrant_cb_group)

        #self.pub = self.create_publisher(SmapPrediction, '/smap_core/perception/predictions', 10, callback_group=self.reentrant_cb_group)


        self.AddPerceptionModule_srv = self.create_service(AddPerceptionModule, 'add_perception_module', self.AddPerceptionModule_callback)
        self.get_logger().info("add_perception_module server online.")
        # Verificar a atomicidade de operações chave

    def AddPerceptionModule_callback(self, request, response):
        self.get_logger().info("Request received.")
        response.is_new=True
        response.success=True
        if( # Check if request is empty or if type is invalid
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
        for cls in self.detectors: # Check if the name of the detector is already registered
            if cls['name'] == request.name:
                response.is_new=False
                response.success = (
                    cls['type'] == request.type and 
                    cls['architecture'] == request.architecture and 
                    cls['n_classes'] == request.n_classes
                )
                eq_classes = 0
                for i in range(cls['n_classes']):
                    print(cls['classes'][i])
                    if cls['classes'][i] == request.classes[i]:
                        eq_classes += 1
                    
                response.success &= (eq_classes == cls['n_classes'])

                if response.success:
                    response.module_id=cls['id']
                    self.get_logger().warning("Client requesting connection with duplicate detector name. The id of the first entry will be sent.")
                else:
                    self.get_logger().warning("Client requesting connection with duplicate detector name and different classes.")
                return response
            
        # Check if the classes list is empty
        if not request.classes:
            self.get_logger().warning("Empty classes list.")
            response.success=False
            return response
        
        # Check the integrity of the classes list
        if len(request.classes) != request.n_classes:
            self.get_logger().warning("Invalid classes list.")
            response.success=False
            return response

        if self.detectors:
            response.module_id=self.detectors[-1]['id']+1
        else:
            response.module_id=1
        self.detectors.append({
            'name': request.name,
            'id': response.module_id,
            'type': request.type,
            'architecture': request.architecture,
            'n_classes': request.n_classes,
            'classes': request.classes
        })
        self.get_logger().info("Request successfully processed.")
        self.get_logger().info("Module added with {} classes:".format(self.detectors[-1]['n_classes']))
        for cl in self.detectors[-1]['classes']:
            self.get_logger().info("\t {}".format(cl))
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