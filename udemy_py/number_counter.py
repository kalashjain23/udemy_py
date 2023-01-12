#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64, String

class NumberCounter(Node):
    
    def __init__(self):
        super().__init__("number_counter")
        self.offset = 2
        
        self.subscriber_ = self.create_subscription(Int64, "number", self.getNumber, 10)
        self.publisher_ = self.create_publisher(String, "number_count", 10)
        self.get_logger().info("Number counter has been started!")
        
    def getNumber(self, msg):
        message_ = String()
        message_.data = f"{msg.data + self.offset}"
        self.offset += 2 
        
        self.publisher_.publish(message_)     

def main(args=None):
    rclpy.init(args=args)
    number_counter = NumberCounter()
    rclpy.spin(number_counter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
