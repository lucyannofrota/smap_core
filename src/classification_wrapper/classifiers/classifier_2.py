#!/usr/bin/env python3

import sys
import rclpy
#sys.path.append(".")
sys.path.append("./..")
from std_srvs.srv import Empty
from std_msgs.msg import String
from classification_wrapper.classification_wrapper import classification_wrapper


class classifier_2(classification_wrapper):

    def __init__(self):
        super().__init__('C_2')
        self.client = self.create_client(Empty, 'test_service', callback_group=self.reentrant_cb_group)
        self.call_timer = self.create_timer(1, self.timer_callback, callback_group=self.reentrant_cb_group)

    #def msg_callback(self,msg):
    #    self.get_logger().info('Rec-2: "%s"' % msg.data)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World-2:'
        self.get_logger().info('Publishing: "%s"' % msg.data)
        _ = self.client.call(Empty.Request())
        self.publisher_.publish(msg)
        self.i += 1