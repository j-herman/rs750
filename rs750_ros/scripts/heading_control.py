#!/usr/bin/env python


import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class HeadingController(Node):

    def __init__(self):
        super().__init__('heading_controller')

        self.rudder_cmd = self.create_publisher(Float64, '/rudder_joint/cmd_pos', 10)
        self.test_cmd = self.create_publisher(Float64, '/test', 10)

        self.heading_sub = self.create_subscription(Float64, '/pose', self.heading_callback, 10)
        self.heading_sub

        self.command_sub = self.create_subscription(Float64, '/cmd_vel', self.command_callback, 10)
        self.command_sub
        
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

        # Controller Parameters
        self.K_heading = 0.025 
        
    def heading_callback(self,msg):
        self.hdg_msg = msg

    def command_callback(self,msg):
        self.des_heading = msg

    def update(self):

        msg2 = Float64()
        try: 
            hdg = self.hdg_msg.data
            goal = self.des_heading.data
            
            err = goal - hdg
            rudder_input = self.K_heading * err
            
            msg2.data = rudder_input
            
            msgtest = Float64()
            msgtest.data = goal

            self.rudder_cmd.publish(msg2)
            self.test_cmd.publish(msgtest)
        
        except:
            return





def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)

    heading_controller = HeadingController()

    rclpy.spin(heading_controller)

    heading_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
