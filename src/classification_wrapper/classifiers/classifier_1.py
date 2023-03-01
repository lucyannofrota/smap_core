#!/usr/bin/env python3

import sys
import rclpy
sys.path.append("./..")
from std_srvs.srv import Empty
from std_msgs.msg import String
from classification_wrapper.classification_wrapper import classification_wrapper


class classifier_1(classification_wrapper):

    def __init__(self):
        super().__init__(1)
        