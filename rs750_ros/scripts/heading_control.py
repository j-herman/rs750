#!/usr/bin/env python


import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from rs750_ros.msg import Control, VesselPose

class HeadingController(Node):

    def __init__(self):
        super().__init__('heading_controller')

        self.rudder_cmd = self.create_publisher(Float64, '/rudder_joint/cmd_pos', 10)

        self.pose_sub = self.create_subscription(VesselPose, '/pose', self.pose_callback, 10)
        self.pose_sub

        self.control_cmd = self.create_publisher(Control, '/control', 10)

        self.control_sub = self.create_subscription(Control, '/control', self.control_callback, 10)
        self.control_sub

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

        # Controller Parameters
        self.K_heading = 0.025

        # Instantiating Variables
        self._app_wind_msg = None
    
    def pose_callback(self,msg):
        self.pose_msg = msg

    def control_callback(self,msg):
        self.control = msg

    def update(self):

        try:
            hdg = self.pose_msg.heading
        except:
            hdg = 0.0

        try:
            goal = self.control.heading
        except:
            goal = 0.0
        
        err = goal - hdg
        
        rudder_input = self.K_heading * err

        msg = Float64()

        msg.data = rudder_input

        self.rudder_cmd.publish(msg)


def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)

    heading_controller = HeadingController()

    rclpy.spin(heading_controller)

    heading_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
