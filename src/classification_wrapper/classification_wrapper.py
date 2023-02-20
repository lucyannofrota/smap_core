#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from semantic_mapping.msg import SmapData

class classification_wrapper(Node):

    def __init__(self,classifier_name='0'):
        super().__init__("smap_classification_{}".format(classifier_name))

        self.reentrant_cb_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(SmapData, '/SMap/classifiers/Data', self.input_data_callback, 10,callback_group=self.reentrant_cb_group)

    def input_data_callback(self,msg):
        pass
        #self.get_logger().info("Data: [{},{},{}]".format(msg.a,msg.b,msg.c))

    def on_process(self): # Pooling
        pass