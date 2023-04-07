#!/usr/bin/env python3

import rclpy
import traceback
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from smap_interfaces.msg import SmapDetections
from smap_interfaces.srv import AddPerceptionModule
from smap_interfaces.srv import SmapClasses

class perception_server(Node):

    detectors={}
    classes={}
    n_classes=0

    def __init__(self):
        super().__init__("classification")

        self.reentrant_cb_group = ReentrantCallbackGroup()

        #self.sub = self.create_subscription(SmapDetections, '/smap_core/perception/detectors/predictions', self.predict, 10, callback_group=self.reentrant_cb_group)

        #self.pub = self.create_publisher(SmapDetections, '/smap_core/perception/predictions', 10, callback_group=self.reentrant_cb_group)


        self.AddPerceptionModule_srv = self.create_service(AddPerceptionModule, '/smap_core/perception_server/add_perception_module', self.AddPerceptionModule_callback,callback_group=self.reentrant_cb_group)
        self.get_logger().info("add_perception_module server online.")

        self.ListClasses_srv = self.create_service(SmapClasses, '/smap_core/perception_server/list_classes', self.ListClasses_callback,callback_group=self.reentrant_cb_group)
        self.get_logger().info("list_classes server online.")

        #SmapClasses
        # Verificar a atomicidade de operações chave

    def AddPerceptionModule_callback(self, request, response):
        self.get_logger().info("add_perception_module request received.")
        self.get_logger().debug("".format(request))
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
            if self.detectors[cls]['name'] == request.name:
                response.is_new=False
                response.success = (
                    self.detectors[cls]['type'] == request.type and 
                    self.detectors[cls]['architecture'] == request.architecture and 
                    self.detectors[cls]['n_classes'] == request.n_classes
                )
                eq_classes = 0
                for i in range(self.detectors[cls]['n_classes']):
                    for cln in request.classes:
                        if self.detectors[cls]['classes'][i] == cln:
                            eq_classes += 1
                            break
                response.success &= (eq_classes == self.detectors[cls]['n_classes'])

                if response.success:
                    response.module_id=self.detectors[cls]['id']
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

        
        response.module_id, new_classes = self.add_detector({
            'name': request.name,
            'id': None,
            'type': request.type,
            'architecture': request.architecture,
            'n_classes': request.n_classes,
            'classes': request.classes
        })

        self.get_logger().info("Request successfully processed.")
        self.get_logger().info("Module added with {} classes:".format(self.detectors[len(self.detectors)]['n_classes']))
        for cl in self.detectors[len(self.detectors)]['classes']:
            is_new=False
            for nc in new_classes:
                if cl == nc:
                    is_new = True
                    break
            if is_new:
                self.get_logger().info("\t\t\t     {} (new)".format(cl))
            else:
                self.get_logger().info("\t\t\t     {}".format(cl))
        
        self.get_logger().info("Server classes [{}]:".format(len(self.classes)))
        for cl in self.classes:
            self.get_logger().info("\t\t    {}".format(self.classes[cl]))
        return response
    
    def add_detector(self,new_detector={}):

        if self.detectors:
            new_detector['id'] = self.detectors[len(self.detectors)]['id']+1
        else:
            new_detector['id'] = 1

        new_classes=[]
        # Combine classes
        if not self.classes:
            for i in range(len(new_detector['classes'])):
                self.classes.update({i:new_detector['classes'][i]})
                self.n_classes += 1
                new_classes.append(new_detector['classes'][i])
        else:
            for i in range(len(new_detector['classes'])):
                # check if exist
                rep = False
                for j in range(self.n_classes):
                    if self.classes[j] == new_detector['classes'][i]:
                        rep = True     

                # append
                if not rep:
                    self.classes.update({len(self.classes):new_detector['classes'][i]})
                    new_classes.append(new_detector['classes'][i])
                    self.n_classes += 1


        self.detectors.update({new_detector['id']:new_detector})
        return new_detector['id'], new_classes

    def ListClasses_callback(self, request, response):
        self.get_logger().info("list_classes request received.")
        self.get_logger().debug("".format(request))
        if request.module_id == 0:
            c_list = self.classes
            response.n_classes = len(c_list)
            response.classes = list(c_list.values())
        else:
            try:
                c_list = self.detectors[request.module_id]
                response.n_classes = c_list['n_classes']
                response.classes = c_list['classes']
            except Exception as e:
                self.get_logger().warn("Requested module does not exist.")
                response.n_classes = 0
                response.classes = []
                return response
            
        return response



    def train(self,data):
        pass

    def predict(self,data):
        print(data)

    def on_process(self): # Pooling
        pass


def main(args=None):

    rclpy.init(args=args)

    class_comp = perception_server()

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