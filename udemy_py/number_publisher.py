#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64

class NumberPublisher(Node):
    
    def __init__(self):
        super().__init__("number_publisher")
        
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(0.5, self.publish_number)
        self.get_logger().info("Number Publisher has been started!")
        
    def publish_number(self):
        message = Int64()
        message.data = 2
        
        self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    number_publisher = NumberPublisher()
    rclpy.spin(number_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()