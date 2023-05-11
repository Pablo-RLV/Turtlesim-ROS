#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Turtle(Node):
    def __init__(self):
        super().__init__('robot_controller')  # nome do nó
        self.twist_msg_ = Twist()  # cria uma variável do tipo Twist
        self.publisher_movement = self.create_publisher(
            Twist, 'cmd_vel', 10)  # cria um publisher
        self.timer_ = self.create_timer(2.0, self.move_robot)  # cria um timer
        self.timer_ = self.create_timer(4.0, self.move_robot_opposite)  # cria um timer

    def move_robot(self):  # função para mover a tartaruga
        self.twist_msg_.linear.x = 0.4
        self.twist_msg_.angular.z = 1.0
        self.publisher_movement.publish(self.twist_msg_)
    def move_robot_opposite(self):  # função para mover a tartaruga
        self.twist_msg_.linear.x = 0.4
        self.twist_msg_.angular.z = -1.0
        self.publisher_movement.publish(self.twist_msg_)

def main(args=None):
    rclpy.init()
    turtle = Turtle()
    rclpy.spin(turtle)
    turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()