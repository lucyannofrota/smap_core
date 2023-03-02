#!/usr/bin/env python3

#import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from semantic_mapping_interfaces.msg import SmapData, SmapPrediction

class classification_wrapper(Node):

    classifier_id = 0
    classes = []
    parameters = []

    def __init__(self,classifier_id=0):
        super().__init__("smap_classification_{}".format(classifier_id))
        self.classifier_id=classifier_id

        self.reentrant_cb_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(SmapData, '/SMap/classifiers/data', self.predict, 10,callback_group= self.reentrant_cb_group)

        self.publisher = self.create_publisher(SmapPrediction, '/SMap/classifiers/predictions', 10,callback_group= self.reentrant_cb_group)

    def train(self,data):
        pass

    def predict(self,data):
        #print(data)
        self.publisher.publish(self.classifier_id)

    def on_process(self): # Pooling
        pass