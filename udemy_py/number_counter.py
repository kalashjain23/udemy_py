#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64

class NumberCounter(Node):
    
    def __init__(self):
        super().__init__("number_counter")
        
        self.subscriber_ = self.create_subscription(Int64, "number", self.getNumber, 10)
        self.get_logger().info("Number counter has been started!")
        
    def getNumber(self, msg):
        self.get_logger().info(f"{msg.data}")        

def main(args=None):
    rclpy.init(args=args)
    number_counter = NumberCounter()
    rclpy.spin(number_counter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
