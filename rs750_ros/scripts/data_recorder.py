#!/usr/bin/env python

import numpy as np
import math
import rclpy
import csv
import os


from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import MagneticField, NavSatFix
from rs750_ros.msg import VesselPose, Control
from datetime import datetime

class DataRecorder(Node):

    def __init__(self):
        super().__init__('data_recorder')

        self.pose_sub = self.create_subscription(VesselPose, '/pose', self.pose_callback, 10)
        self.pose_sub

        self.navsat_sub = self.create_subscription(NavSatFix, '/rs750/navsat', self.navsat_callback,
            10)
        self.navsat_sub  # prevent unused variable warning

        self.heading_sub = self.create_subscription(MagneticField, '/magnetometer', self.heading_callback, 10)
        self.heading_sub

        self.mainsail_sub = self.create_subscription(Float64, '/main_sail_joint/cmd_pos', self.mainsail_callback, 10)
        self.mainsail_sub

        self.power_sub = self.create_subscription(Float32, '/ges/wattage', self.power_callback,
            10)
        self.power_sub  # prevent unused variable warning

        self.drag_sub = self.create_subscription(Float32, '/ges/drag', self.drag_callback,
            10)
        self.drag_sub  # prevent unused variable warning

        self.control_cmd = self.create_publisher(Control, '/control', 10)

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

        # test_time = 30  # Time of run in sec
        # self.test_timer = self.create_timer(test_time, self.update_course)

        self.pose_msg = None
        self.navsat_msg = None
        self.power_msg = None

        self.test_routine = [50., 60., 70., 90., 110., 130., 150., 170., -170., -150., -130., -110., -90., -70., -50.]
        # [50., 60., 70., 80., 90., 100., 110., 120., 125., 130., 140., 150., 160.]
        # Test: Angle of Attack Sweep
        # [0.,5.,10.,15.,20.,25.,30.,35.,40.,45.]
        # Test: Heading Sweep
        # [60., 65., 70., 75., 80., 85., 90., 95., 100., 105., 110., 115., 120., 125., 130., 135., 140., 145., 150., 155., 160., 165., 170.]

        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")

        self.csv_file_path = os.path.join(os.path.expanduser('~'), 'Documents/MATLAB', f'data_{timestamp}.csv')


        self.csv_file = open(self.csv_file_path, mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time','Heading','AppWind','Velocity','Lat','Long','Power','Drag','Test Value','mx','my','SailAngle'])  # Write header to CSV file

    def __del__(self):
        self.csv_file.close()

    def pose_callback(self, msg):
        self.pose_msg = msg

    def navsat_callback(self, msg):
        self.navsat_msg = msg

    def power_callback(self, msg):
        self.power_msg = msg

    def drag_callback(self, msg):
        self.drag_msg = msg

    def heading_callback(self, msg):
        self.mag_field = msg.magnetic_field

    def mainsail_callback(self, msg):
        self.sailtrim = msg

    def update(self):
        try:
            data0 = self.navsat_msg.header.stamp.sec + self.navsat_msg.header.stamp.nanosec / 1000000000
        except:
            data0 = 0.

        try:
            data1 = self.pose_msg.heading
        except:
            data1 = 0.
            
        try:
            data2 = self.pose_msg.app_wind
        except:
            data2 = 0.
        
        try:
            data3 = self.pose_msg.linear_v
        except:
            data3 = 0.
        
        try:
            data4 = self.navsat_msg.latitude
        except:
            data4=0.

        try:
            data5 = self.navsat_msg.longitude
        except:
            data5 = 0.
        
        try:
            data6 = self.power_msg.data
        except:
            data6 = 0.
        
        try:
            data7 = self.drag_msg.data
        except:
            data7 = 0.
        
        try:
            data8 = self.test_routine[0]
        except:
            data8= 0.

        try:
            data9 = self.mag_field.x
        except:
            data9= 0.

        try:
            data10 = self.mag_field.y
        except:
            data10= 0.

        try:
            data11 = self.sailtrim.data
        except:
            data11= 0.

        self.csv_writer.writerow([data0, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11])

    def update_course(self):

        if not self.test_routine:
            self.get_logger().info("Test Complete")
            DataRecorder().destroy_node()
            rclpy.shutdown()

        control_msg = Control()
        control_msg.autotrim = True
        control_msg.angle_of_attack = 10. #self.test_routine[0]
        control_msg.heading = self.test_routine[0]

        self.get_logger().info("Changing...")

        self.test_routine.pop(0)

        self.control_cmd.publish(control_msg)

def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)
 
    data_recorder = DataRecorder()

    rclpy.spin(data_recorder)

    data_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()