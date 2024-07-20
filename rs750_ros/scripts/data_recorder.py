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

        now = datetime.now()
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_file_path = os.path.join(os.path.expanduser('~'), 'Documents/MATLAB', f'data_{timestamp}.csv')
        self.csv_file = open(self.csv_file_path, mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time','Heading','AppWind','Velocity','Lat','Long','Power','Drag','Test Value','Zeta','NA','SailAngle'])  # Write header to CSV file

        # Subscribers
        self.pose_sub = self.create_subscription(VesselPose, '/pose', self.pose_callback, 10)
        self.pose_sub

        self.navsat_sub = self.create_subscription(NavSatFix, '/rs750/navsat', self.navsat_callback,
            10)
        self.navsat_sub  # prevent unused variable warning

        self.heading_sub = self.create_subscription(Float32, '/ges/debugF', self.heading_callback, 10)
        self.heading_sub

        self.mainsail_sub = self.create_subscription(Float64, '/main_sail_joint/cmd_pos', self.mainsail_callback, 10)
        self.mainsail_sub

        self.power_sub = self.create_subscription(Float32, '/ges/wattage', self.power_callback,
            10)
        self.power_sub  # prevent unused variable warning

        self.drag_sub = self.create_subscription(Float32, '/ges/drag', self.drag_callback,
            10)
        self.drag_sub  # prevent unused variable warning

        self.control_sub = self.create_subscription(Control, '/control', self.control_callback,
            10)
        self.control_sub  # prevent unused variable warning


        # Publishers
        self.control_cmd = self.create_publisher(Control, '/control', 10)

        self.zeta_cmd = self.create_publisher(Float32, '/ges/zeta', 10)

        # Update Timer
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

        # Set Up Test Timer
        test_time = 40  # Time of run in sec
        # self.test_timer = self.create_timer(test_time, self.heading_sweep)
        self.test_timer = self.create_timer(test_time, self.comboTest)

        self.pose_msg = None
        self.navsat_msg = None
        self.power_msg = None

        # Set Baseline Value -  These values are fixed during a sweep of another
        self.aoa_baseline = 10.
        self.heading_baseline = 70.
        self.zeta_baseline = 0.781

        # Set Routine
        self.heading_routine = [50., 70., 90., 110., 130., 150., 170.]
        self.zeta_routine = [0.1, 0.5, 0.8]
        self.aoa_routine = [10.,12.,15.,20.,25.,30.]

        self.makeNewroutine()

        # self.test_routine = [40., 100., 160., 160., 100., 40.]
        # self.zeta_routine = [0.1, 0.4, 0.9]
        # self.test_routine = [40., 60., 80., 100., 120., 140., 160., 160., 140., 120., 100., 80., 60., 40.]
        # self.zeta_routine = [0.1, 0.4, 0.9]
        # [10.,12.,15.,20.,25.,30.,]
        # [50., 70., 90., 110., 130., 150., 170.]
        # [50., 70., 90., 110., 130., 150., 170., -170., -150., -130., -110., -90., -70., -50.]
        # [50., 60., 70., 80., 90., 100., 110., 120., 125., 130., 140., 150., 160., 170.]
        # [0.,5.,10.,15.,20.,25.,30.,35.,40.,45.]
        # [60., 65., 70., 75., 80., 85., 90., 95., 100., 105., 110., 115., 120., 125., 130., 135., 140., 145., 150., 155., 160., 165., 170.]

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
        self.true_heading = msg

    def mainsail_callback(self, msg):
        self.sailtrim = msg
    
    def control_callback(self, msg):
        self.control_input = msg

    def heading_sweep(self):
        if not self.heading_routine:
            self.get_logger().info("Heading Test Complete")
            DataRecorder().destroy_node()
            rclpy.shutdown()

        control_msg = Control()
        control_msg.autotrim = True
        control_msg.angle_of_attack = self.aoa_baseline
        control_msg.heading = self.heading_routine[0]
        
        self.control_cmd.publish(control_msg)
        self.get_logger().info("Changing Heading")

        zeta_msg = Float32()
        zeta_msg.data = self.zeta_baseline
        self.zeta_cmd.publish(zeta_msg)
        
        self.heading_routine.pop(0)
    
    def zeta_sweep(self):
        if not self.zeta_routine:
            self.get_logger().info("Zeta Test Complete")
            DataRecorder().destroy_node()
            rclpy.shutdown()

        zeta_msg = Float32()
        zeta_msg.data = self.zeta_routine[0]
        self.zeta_cmd.publish(zeta_msg)
        self.get_logger().info("Changing Zeta")
        self.zeta_routine.pop(0)

    def aoa_sweep(self):

        if not self.aoa_routine:
            self.get_logger().info("AOA Test Complete")
            DataRecorder().destroy_node()
            rclpy.shutdown()

        control_msg = Control()
        control_msg.autotrim = True
        control_msg.angle_of_attack = self.aoa_routine[0]
        control_msg.heading = self.heading_baseline
        
        self.control_cmd.publish(control_msg)
        self.get_logger().info("Changing AoA")

        zeta_msg = Float32()
        zeta_msg.data = self.zeta_baseline
        self.zeta_cmd.publish(zeta_msg)
        
        self.aoa_routine.pop(0)

    def comboTest(self):
        control_msg = Control()
        control_msg.autotrim = True
        control_msg.angle_of_attack = self.aoa_baseline
        control_msg.heading = self.newHeadingroutine[0]
        self.control_cmd.publish(control_msg)

        zeta_msg = Float32()
        zeta_msg.data = self.newZetaroutine[0]
        self.zeta_cmd.publish(zeta_msg)

        self.get_logger().info("Changing Heading and Zeta")
        
        self.newHeadingroutine.pop(0)
        self.newZetaroutine.pop(0)

        if len(self.newHeadingroutine) == 0:
            self.get_logger().info("Test Complete")
            DataRecorder().destroy_node()
            rclpy.shutdown()

            return

    def makeNewroutine(self):
        self.newHeadingroutine = self.heading_routine
        self.newZetaroutine = self.zeta_routine

        self.newZetaroutine = [element for element in self.newZetaroutine for _ in range(len(self.heading_routine))]

        for x in range(len(self.zeta_routine)-1):
            self.newHeadingroutine += self.newHeadingroutine[::-1]


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
            data8 = self.control_input.heading
        except:
            data8= 0.

        try:
            data9 = self.control_input.angle_of_attack
        except:
            data9= 0.

        try:
            data10 = 0.
        except:
            data10= 0.

        try:
            data11 = self.sailtrim.data
        except:
            data11= 0.

        self.csv_writer.writerow([data0, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11])    

def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)
 
    data_recorder = DataRecorder()

    rclpy.spin(data_recorder)

    data_recorder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()