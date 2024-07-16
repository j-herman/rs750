#!/usr/bin/env python

# Copyright (C) 2024 Jessica Herman
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

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TwistToRudder(Node):

    def __init__(self):
        super().__init__('twist_to_rudder')

        self.publisher_ = self.create_publisher(Float64, '/rudder_joint/cmd_pos', 10)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        msg2 = Float64()
        msg2.data = msg.angular.z * math.pi / 2
        self.publisher_.publish(msg2)


def main(args=None):
    rclpy.init(args=args)

    twist_to_rudder = TwistToRudder()

    rclpy.spin(twist_to_rudder)

    twist_to_rudder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()