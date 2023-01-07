#!/usr/bin/env python

import rclpy
from rclpy.node import Node

def main(args = None):
    rclpy.init(args=args)
    node = Node("basic_python")
    node.get_logger().info("Hello World")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
