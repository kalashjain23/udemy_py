#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self.action_client_ = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )
        
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.action = order
        
        self.action_client_.wait_for_server()
        
        self.send_goal_future_ = self.action_client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        self.send_goal_future_.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        
        self.get_logger().info('Goal accepted.')
        
        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.partial_sequence}")

def main(args=None):
    rclpy.init(args=args)
    
    action_client = FibonacciActionClient()
    
    action_client.send_goal(10)
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
