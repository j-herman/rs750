#!/usr/bin/env python

import numpy as np
import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from rs750_ros.msg import VesselPose

class PoseGenerator(Node):

    def __init__(self):
        super().__init__('pose_generator')
        
        self.pose_pub = self.create_publisher(VesselPose, '/pose', 10)

        self.heading_sub = self.create_subscription(Imu, '/rs750/IMU', self.heading_callback, 10)
        self.heading_sub

        self.wind_sub = self.create_subscription(Vector3, '/wind/apparent', self.wind_callback,
            10)
        self.wind_sub  # prevent unused variable warning

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

        self.quat = None
        self._app_wind_msg = None

    def heading_callback(self, msg):
        self.quat = msg.orientation

    def wind_callback(self, msg):
        self._app_wind_msg = msg

    def update(self):

        # Heading Section
        if self.quat == None:
            self.get_logger().info('no pose message', once=True)
            return
        
        q = self.quat
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        
        yaw = np.arctan2(siny_cosp, cosy_cosp) * 180 / np.pi
        
        msg1 = VesselPose()
        msg1.heading = yaw 

        # Apparent Wind Reading
        if self._app_wind_msg == None:
            self.get_logger().info('no wind message', once=True)
            return
        wind_vel = self._app_wind_msg

        wind_speed = math.sqrt(
            wind_vel.x * wind_vel.x
            + wind_vel.y * wind_vel.y
            + wind_vel.z * wind_vel.z)
        #self.get_logger().info(f'In loop: wind speed is {wind_speed}')
        # For low wind speeds the angle will be noisy. 
        wind_angle = 0.0
        if wind_speed > 0.01:
            # Measure wind angle between (-1 * wind_vel) and positive x-axis
            wind_angle = math.atan2(-wind_vel.y, -wind_vel.x)

        msg1.app_wind = wind_angle *math.pi / 180


        # Linear Velocity Section

        self.pose_pub.publish(msg1)

def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)
 
    pose_generator = PoseGenerator()

    rclpy.spin(pose_generator)

    pose_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()