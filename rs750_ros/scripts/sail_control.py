#!/usr/bin/env python

# Copyright (C) 2020  Rhys Mainwaring
# Copyright (C) 2024  Jessica Herman
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Migrates https://github.com/srmainwaring/rs750/blob/gazebo11/rs750_controller/nodes/sail_controller to ROS2 


import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from rs750_ros.msg import Control,VesselPose


class SailController(Node):

    def __init__(self):
        super().__init__('sail_controller')

        # Parameters (TODO)
        self._sail_angle_attack_deg = 10.
        self._sail_angle_min_deg = 10.
        self._sail_angle_max_deg = 90.

        self._sail_angle_attack = self._rad(self._sail_angle_attack_deg)
        self._sail_angle_min = self._rad(self._sail_angle_min_deg)
        self._sail_angle_max = self._rad(self._sail_angle_max_deg)

        self._app_wind_msg = None

        self.mainsail_pub = self.create_publisher(Float64, '/main_sail_joint/cmd_pos', 10)
        self.foresail_pub = self.create_publisher(Float64, '/fore_sail_joint/cmd_pos', 10)
        self.wind_sub = self.create_subscription(Vector3, '/wind/apparent', self.wind_callback,
            10)
        self.wind_sub  # prevent unused variable warning
        
        self.autotrim_sub = self.create_subscription(Control, '/control', self.control_callback, 10)
        self.autotrim_sub

        self.autotrim = True

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.update)

    def wind_callback(self, msg):
        self._app_wind_msg = msg
    
    def control_callback(self, msg):
         self.control_msg = msg
         self.autotrim = self.control_msg.autotrim

    def update(self):
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

        # Set the sail trim
        sail_angle = 0.0
        if wind_angle > 0.0:
            # wind_angle > 0, sail_angle > 0
            sail_angle = wind_angle - self._sail_angle_attack
            
            # Apply bounds
            sail_angle = max(self._sail_angle_min, sail_angle)
            sail_angle = min(self._sail_angle_max, sail_angle)
        else:
            # wind_angle < 0, sail_angle < 0
            sail_angle = wind_angle + self._sail_angle_attack

            # Apply bounds
            sail_angle = min(-self._sail_angle_min, sail_angle)
            sail_angle = max(-self._sail_angle_max, sail_angle)

        msg = Float64()

        msg.data = sail_angle

        if self.autotrim:
            self.mainsail_pub.publish(msg)
            self.foresail_pub.publish(msg)
        else:
            return

    def _deg(self, rad):
        return rad * 180. / math.pi

    def _rad(self, deg):
        return deg * math.pi / 180.

def main(args=None):
    '''ROS node for sail controller'''
    rclpy.init(args=args)

    sail_controller = SailController()

    rclpy.spin(sail_controller)

    sail_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



