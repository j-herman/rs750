#!/usr/bin/env python
# cmd_publisher.py


import sys
import rclpy
from rclpy.node import Node
from rs750_ros.msg import Control

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.publisher_ = self.create_publisher(Control, '/control', 10)
        self.get_logger().info('Publisher node has started. Waiting for input...')
        self.publish_input_value()

    def publish_input_value(self):
        while True:
            user_input = input("Enter a float value to publish or 'exit' to quit: ")
            if user_input.lower() == 'exit':
                self.get_logger().info('Exiting...')
                break
            try:
                value = float(user_input)
                msg = Control()
                msg.autotrim = True
                msg.heading = value
                msg.angle_of_attack = 10.
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {msg.heading}')
            except ValueError:
                self.get_logger().error("The input value must be a float.")

def main(args=None):
    rclpy.init(args=args)
    node = CmdPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
