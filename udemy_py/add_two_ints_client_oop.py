#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):
    
    def __init__(self):
        super().__init__("node_name")
        self.call_add_two_ints(5, 8)
        
    def call_add_two_ints(self, a, b):
        client_ = self.create_client(AddTwoInts, "add_two_ints")
        while not client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to add ints")
            
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = client_.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
        
    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"a: {a}, b: {b}\nSum: {response.sum}")
        except Exception as e:
            self.get_logger().error(e)

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
