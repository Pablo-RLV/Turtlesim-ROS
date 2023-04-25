#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Any
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv._set_parameters import SetParameters
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.twist_msg_ = Twist()
        self.publisher_movement = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.move_turtle)
        self.rainbow = {'red': [255, 0, 0],
                        'orange': [255, 127, 0],
                        'yellow': [255, 255, 0],
                        'green': [0, 255, 0],
                        'blue': [0, 0, 255],
                        'indigo': [75, 0, 130],
                        'violet': [143, 0, 255]
                        }
        self.contador = 0
        self.cli = self.create_client(SetParameters, "turtlesim/set_parameters")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()

    def move_turtle(self):
        self.contador += 1
        if self.contador < 45:
            self.twist_msg_.linear.x = 0.001 + self.contador * 0.11
            self.twist_msg_.angular.z = 1.0
            self.publisher_movement.publish(self.twist_msg_)
            self.change_color()
    
    def color_line(self, r, g, b):
        client = self.create_client(SetPen, 'turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 2
        request.off = False
        future = client.call_async(request)
        if future.result() is not None:
            self.get_logger().info('Result of set_pen: %s' % future.result().success)
        else:
            pass
    
    def change_color(self):
        if self.contador < 6:
            self.color_line(self.rainbow['red'][0], self.rainbow['red'][1], self.rainbow['red'][2])
        elif self.contador < 12:
            self.color_line(self.rainbow['orange'][0], self.rainbow['orange'][1], self.rainbow['orange'][2])
        elif self.contador < 18:
            self.color_line(self.rainbow['yellow'][0], self.rainbow['yellow'][1], self.rainbow['yellow'][2])
        elif self.contador < 24:
            self.color_line(self.rainbow['green'][0], self.rainbow['green'][1], self.rainbow['green'][2])
        elif self.contador < 30:
            self.color_line(self.rainbow['blue'][0], self.rainbow['blue'][1], self.rainbow['blue'][2])
        elif self.contador < 36:
            self.color_line(self.rainbow['indigo'][0], self.rainbow['indigo'][1], self.rainbow['indigo'][2])
        else:
            self.color_line(self.rainbow['violet'][0], self.rainbow['violet'][1], self.rainbow['violet'][2])
        self.set_rgb(self.contador * 4,self.contador * 4, self.contador * 4)

    def set_rgb(self, r: int, g: int, b: int) -> None:
        self.send_request("background_r", r)
        self.send_request("background_g", g)
        self.send_request("background_b", b)

    def send_request(self, param_name, param_value):
        if isinstance(param_value, float):
            val = ParameterValue(double_value=param_value, type=ParameterType.PARAMETER_DOUBLE)
        elif isinstance(param_value, int):
            val = ParameterValue(integer_value=param_value, type=ParameterType.PARAMETER_INTEGER)
        elif isinstance(param_value, str):
            val = ParameterValue(string_value=param_value, type=ParameterType.PARAMETER_STRING)
        elif isinstance(param_value, bool):
            val = ParameterValue(bool_value=param_value, type=ParameterType.PARAMETER_BOOL)
        self.req.parameters = [ParameterMsg(name=param_name, value=val)]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init()
    turtle = Turtle()
    rclpy.spin(turtle)
    turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()