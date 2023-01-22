#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

from udemy_interfaces.srv import SetLed

class BatteryNode(Node):
    
    def __init__(self):
        super().__init__("battery_node")
        self.battery_ = "full"
        self.last_time_battery_was_changed_ = self.get_current_time()
        self.timer_ = self.create_timer(0.1, self.check_battery_state)
        
    def get_current_time(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0
        
    def check_battery_state(self):
        time_now = self.get_current_time()
        if self.battery_ == "full":
            if time_now - self.last_time_battery_was_changed_ > 4.0:
                self.battery_ = "empty"
                self.get_logger().info("Battery is now empty! Charging the battery...")
                self.last_time_battery_was_changed_ = time_now
                self.call_set_led(2, "on")
        else:
            if time_now - self.last_time_battery_was_changed_ > 6.0:
                self.battery_ = "full"
                self.get_logger().info("Battery is full!")
                self.last_time_battery_was_changed_ = time_now
                self.call_set_led(2, "off")
        
    def call_set_led(self, led_num, state):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the service!")
        
        request = SetLed.Request()
        request.led_num = led_num
        request.state = state
        
        future = client.call_async(request=request)
        future.add_done_callback(partial(self.callback_call_set_led, led_num=led_num, state=state)) 
        
    def callback_call_set_led(self, future, led_num, state):
        try:
            response = future.result()
            self.get_logger().info(f"LED Number: {led_num}, State: {state} ==> Response: {response.success}")
        except Exception as e:
            self.get_logger().error(e)
         

def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryNode()
    rclpy.spin(battery_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
