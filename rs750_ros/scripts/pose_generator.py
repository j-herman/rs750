#!/usr/bin/env python

import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class PoseGenerator(Node):

    def __init__(self):
        super().__init__('pose_generator')
        
        self.pose_pub = self.create_publisher(Float64, '/pose', 10)

        self.heading_sub = self.create_subscription(Imu, '/rs750/IMU', self.heading_callback, 10)
        self.heading_sub
    
    def heading_callback(self, msg):
        
        q = msg.orientation

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        
        yaw = np.arctan2(siny_cosp, cosy_cosp) * 180 / np.pi
        
        msg2 = Float64()
        msg2.data = yaw 
        self.pose_pub.publish(msg2)



def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)
 
    pose_generator = PoseGenerator()

    rclpy.spin(pose_generator)

    pose_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()