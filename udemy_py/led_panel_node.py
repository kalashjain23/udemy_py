#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from udemy_interfaces.srv import SetLed
from udemy_interfaces.msg import LedStatus

class LedPanelNode(Node):
    
    def __init__(self):
        super().__init__("led_panel_node")
        self.declare_parameter("panel_state", [0,0,0])
        
        self.led_panel = self.get_parameter("panel_state").value
        self.server_ = self.create_service(SetLed, "set_led", self.callback_led_panel)
        self.publisher_ = self.create_publisher(LedStatus, "led_panel_state", 10)
        self.timer_ = self.create_timer(1.0, self.publish_panel_state)
        
        self.get_logger().info("LED_Panel server is started!")
        
    def callback_led_panel(self, request, response):
        if request.led_num > 3 or request.led_num <= 0:
            self.get_logger().error("Not a valid LED point!")
            response.success = False
            return response
        
        if request.state not in ["off", "on"]:
            self.get_logger().error("Not a valid state!")
            response.success = False
            return response
        
        if request.state == "on":
            self.led_panel[request.led_num] = 1
        else:
            self.led_panel[request.led_num] = 0
        
        response.success = True
        self.publish_panel_state()
        return response
            
    def publish_panel_state(self):
        panel_status = LedStatus()
        panel_status.status = self.led_panel
        self.publisher_.publish(panel_status)        

def main(args=None):
    rclpy.init(args=args)
    led_panel_node = LedPanelNode()
    rclpy.spin(led_panel_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
