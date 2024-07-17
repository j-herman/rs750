#!/usr/bin/env python

import numpy as np
import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from rs750_ros.msg import VesselPose

class PoseGenerator(Node):

    def __init__(self):
        super().__init__('pose_generator')
        
        self.pose_pub = self.create_publisher(VesselPose, '/pose', 10)

        self.time_pub = self.create_publisher(Float64, '/sim_time', 10)

        self.heading_sub = self.create_subscription(MagneticField, '/magnetometer', self.heading_callback, 10)
        self.heading_sub

        self.wind_sub = self.create_subscription(Vector3, '/wind/apparent', self.wind_callback,
            10)
        self.wind_sub  # prevent unused variable warning

        self.navsat_sub = self.create_subscription(NavSatFix, '/rs750/navsat', self.navsat_callback,
            10)
        self.navsat_sub  # prevent unused variable warning

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

        self.quat = None
        self._app_wind_msg = None
        self.mag_field = None
        self.navsat_msg = None
        self.sim_time = None
        self.lat_array = [0, 0, 0, 0, 0]
        self.long_array = [0, 0, 0, 0, 0]
        self.dt = [0, 0, 0, 0, 0]

    def heading_callback(self, msg):
        self.mag_field = msg.magnetic_field

    def wind_callback(self, msg):
        self._app_wind_msg = msg

    def navsat_callback(self, msg):
        self.navsat_msg = msg
        self.sim_time = self.navsat_msg.header.stamp.sec + self.navsat_msg.header.stamp.nanosec / 1000000000
        msg_time = Float64()
        msg_time.data = self.sim_time
        self.time_pub.publish(msg_time)

    def update(self):

        # Heading Section
        if self.mag_field == None:
            self.get_logger().info('no mag field message', once=True)
            return
        magnetic_field = self.mag_field
        
        try:
            p = (math.atan2(magnetic_field.y,magnetic_field.x) + math.pi/2)*180/math.pi
            if p > 180:
                q = p-360
            else:
                q = p
            yaw = q
        except:
            yaw = 0.
        
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

        msg1.app_wind = wind_angle * 180 / math.pi


        # Linear Velocity Section
        if self.navsat_msg == None:
            self.get_logger().info('no navsat message', once=True)
            return
        
        self.lat_array.append(self.navsat_msg.latitude)
        self.lat_array.pop(0)
        self.long_array.append(self.navsat_msg.longitude)
        self.long_array.pop(0)
        self.dt.append(self.sim_time)
        self.dt.pop(0)

        lat1 = self.lat_array[0]
        lat2 = self.lat_array[4]
        lon1 = self.long_array[0]
        lon2 = self.long_array[4]
        deltat = self.dt[4] - self.dt[0]
        
        R = 6378.137
        dLat = lat2 * math.pi / 180 - lat1 * math.pi / 180
        dLon = lon2 * math.pi / 180 - lon1 * math.pi / 180
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R * c * 1000 # meters

        if deltat == 0:
            msg.linear_v = 0
        else:
            msg1.linear_v = d / deltat

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