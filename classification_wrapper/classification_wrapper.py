#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class classification_wrapper(Node):

    def __init__(self,classifier_name='0'):
        super().__init__("smap_classification_{}".format(classifier_name))

        self.reentrant_cb_group = ReentrantCallbackGroup()
        # Recriar o exemplo de: https://docs.ros.org/en/foxy/How-To-Guides/Using-callback-groups.html

        #self.publisher_ = self.create_publisher(String, '/SMap/classifiers/'+classifier_name, 10,callback_group=self.reentrant_cb_group)
        #self.create_subscription(String, '/SMap/classifiers/'+classifier_name,self.msg_callback,10,callback_group=self.reentrant_cb_group)
        #self.subscription_ = self.create_publisher(String, '/SMap/classifiers/'+classifier_name, 10,callback_group=self.reentrant_cb_group)
        #timer_period = 1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.reentrant_cb_group)
        #self.i = 0

    def msg_callback(self,msg):
        self.get_logger().info('Rec-Default: "%s"' % msg.data)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def on_process(self): # Pooling
        pass