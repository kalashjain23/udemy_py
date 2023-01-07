#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class BasicNode(Node):
    
    def __init__(self):
        self.counter = 0
        super().__init__("basic_python")
        self.get_logger().info("Hello World")
        self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f"Hello {self.counter}")
        
def main(args = None):
    rclpy.init(args=args)
    node = BasicNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
