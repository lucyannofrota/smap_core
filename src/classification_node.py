#!/usr/bin/env python3

# This is a dummy file. The complete node can be generated using a launch file like the example: "importer_example_launch.py" .

# TODO ##url## to the node launch example

import rclpy

def main(args=None):

    rclpy.init(args=args)

    node = rclpy.create_node("classification_dummy")
    # TODO Insert url example
    node.get_logger().fatal("No classifiers detected!\nThe classifiers shoud be define using a launch file. Example: #url#")
    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    main()