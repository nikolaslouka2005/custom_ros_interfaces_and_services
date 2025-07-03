#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed
 
class LedPanel(Node):
    def __init__(self):
        super().__init__("led_panel") 
        self.led_panel_array = [0, 0, 0]
        self.led_states_pub_ = self.create_publisher(LedStateArray, "led_panel_state", 10)
        self.server_ = self.create_service(SetLed, "set_led",self.callback_self_led)
        self.create_timer(5.0, self.publish_led_states)
        self.get_logger().info("Led panel node started!")

    def publish_led_states(self):
        msg = LedStateArray()
        msg.led_states = self.led_panel_array
        self.led_states_pub_.publish(msg)
        self.get_logger().info(f"[TIMER] Published: {msg.led_states}")
    
    def callback_self_led(self, request: SetLed.Request, response: SetLed.Response): 
        self.led_panel_array[request.led_number-1] = request.state
        self.publish_led_states()
        response.success = True
        return response

 
def main(args=None):  
    rclpy.init(args=args)
    node = LedPanel() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
