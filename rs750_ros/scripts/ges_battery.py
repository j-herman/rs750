#!/usr/bin/env python

import numpy as np
import math
import rclpy


from rclpy.node import Node
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix

class GESBattery(Node):

    def __init__(self):
        super().__init__('ges_battery')

        self.navsat_sub = self.create_subscription(NavSatFix, '/rs750/navsat', self.navsat_callback,
            10)
        self.navsat_sub  # prevent unused variable warning

        self.power_sub = self.create_subscription(Float32, '/ges/wattage', self.power_callback,
            10)
        self.power_sub  # prevent unused variable warning

        self.energy_pub = self.create_publisher(Float64, '/BattStat', 10)

        self.timer_period = 3.
        self.timer = self.create_timer(self.timer_period, self.update)

        self.navsat_msg = None
        self.power_msg = None
        
        self.time = [0., 0.]
        self.TotalEnergy = 0.0

    def navsat_callback(self, msg):
        self.navsat_msg = msg

    def power_callback(self, msg):
        self.power_msg = msg

        try:
            self.time.append(self.navsat_msg.header.stamp.sec + self.navsat_msg.header.stamp.nanosec / 1000000000)
        except:
            self.time.append(0.)
        self.time.pop(0)
        
        dt = self.time[1] - self.time[0]

        try:
            power = self.power_msg.data
        except:
            power = 0.

        Energy = power*dt

        self.TotalEnergy += Energy

    def update(self):
        WattHours = self.TotalEnergy/3600
        msg = Float64()
        msg.data = WattHours
        self.energy_pub.publish(msg)



def main(args=None):
    '''ROS node for battery'''
    rclpy.init(args=args)
 
    ges_battery = GESBattery()

    rclpy.spin(ges_battery)

    ges_battery.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()