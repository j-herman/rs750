#!/usr/bin/env python

import math
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion
from std_msgs.msg import Float64, String
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

        self.mainsail_pub = self.create_publisher(Float64, '/main_sail_joint/cmd_pos', 10)
        self.foresail_pub = self.create_publisher(Float64, '/fore_sail_joint/cmd_pos', 10)

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.update)

        self.test_cmd = self.create_publisher(Float64, '/test', 10)

        # Controller Parameters
        self.K_heading = 0.0025

    def pose_callback(self,msg):
        self.pose_msg = msg

    def control_callback(self,msg):
        self.control = msg

    def _deg(self, rad):
        return rad * 180. / math.pi

    def _rad(self, deg):
        return deg * math.pi / 180.
    
    def deltar(self,hdg,goal):
        rmax = math.pi/4
        err = self._rad(hdg - goal)

        if math.cos(err) >= 0:
            return -1*rmax*math.sin(err)
        else:
            return -1*rmax*np.sign(math.sin(err))

    def findQuad(self,angle):
        if angle > 0 and angle < 90:
            return 1
        if angle >= 90 and angle <= 180:
            return 2
        if angle >= -180 and angle <= -90:
            return 3
        else:
            return 4
        
    def findMode(self,goal,hdg):
        goalQuad = self.findQuad(goal)
        hdgQuad = self.findQuad(hdg)

        if goalQuad == hdgQuad or goalQuad + hdgQuad == 3 or goalQuad + hdgQuad == 7:
            mode = "trim"
        if goalQuad == 1 and hdgQuad == 4:
            mode = "tack"
        if goalQuad == 4 and hdgQuad == 1:
            mode = "tack"
        else:
            mode = "trim"
        self.mode = mode

    def update(self):

        try:
            hdg = self.pose_msg.heading
        except:
            hdg = 70.0

        try:
            goal = self.control.heading
        except:
            goal = 70.0

        try:
            aoa = self.control.angle_of_attack
        except:
            aoa = 10.0

        testmsg = Float64()
        msg2 = Control()
        msg = Float64()
        msg_sail = Float64()

        self.findMode(goal,hdg)

        match self.mode:
            case "trim":
                rudder_input = self.deltar(hdg,goal)
                msg.data = rudder_input

                msg2.autotrim = True
                msg2.angle_of_attack = aoa
                msg2.heading = goal
                self.get_logger().info("Trimming...")

            case "tack":
                tackgoal = hdg + np.sign(goal)*40
                msg2.autotrim = False
                
                rudder_input = self.deltar(hdg,tackgoal)

                
                msg2.angle_of_attack = aoa
                msg2.heading = goal

                msg.data = rudder_input
                testmsg.data = tackgoal

                self.get_logger().info("Tacking...")
            
        self.test_cmd.publish(testmsg)
        self.rudder_cmd.publish(msg)
        self.control_cmd.publish(msg2)


def main(args=None):
    '''ROS node for heading controller'''
    rclpy.init(args=args)

    heading_controller = HeadingController()

    rclpy.spin(heading_controller)

    heading_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
