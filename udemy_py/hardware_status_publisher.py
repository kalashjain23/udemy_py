#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from udemy_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):
    
    def __init__(self):
        super().__init__("hardware_status_publisher")
        
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status_publisher", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hardware_status)
        self.get_logger().info("Hardware status publisher has been started!")
        
    def publish_hardware_status(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special!"
        
        self.publisher_.publish(msg)        

def main(args=None):
    rclpy.init(args=args)
    hardware_status_publisher_node = HardwareStatusPublisherNode()
    rclpy.spin(hardware_status_publisher_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()