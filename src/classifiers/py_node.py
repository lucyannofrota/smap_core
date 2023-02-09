#!/usr/bin/env_python3

import rclpy
from classifier_wrapper_node import Classifier_Wrapper

def main(args=None):
	rclpy.init(args=args)
	node = Classifier_Wrapper()
	rclpy.spin(node)
	rclpy.shutdown()

if name == "__main__":
	main()