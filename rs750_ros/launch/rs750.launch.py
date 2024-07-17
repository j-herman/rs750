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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    #pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rs750 = get_package_share_directory('rs750_gazebo')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-v4 -r sailing_course_ges.sdf'
        }.items(),
    )
    sdf_file = os.path.join(pkg_rs750, 'models', 'rs750', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/rudder_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/fore_sail_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/main_sail_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/ges/zeta@std_msgs/msg/Float32]gz.msgs.Float',
            '/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer',
            '/world/waves/model/rs750/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/waves/model/rs750/link/base_link/sensor/anemometer/anemometer@geometry_msgs/msg/Vector3[gz.msgs.Vector3d',
            '/world/waves/model/rs750/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/ges/wattage@std_msgs/msg/Float32[gz.msgs.Float',
            '/ges/drag@std_msgs/msg/Float32[gz.msgs.Float',
            '/ges/xdot@std_msgs/msg/Float32[gz.msgs.Float',
            '/ges/ydot@std_msgs/msg/Float32[gz.msgs.Float'

        ],
        remappings=[
            ('/world/waves/model/rs750/joint_state', '/joint_states'),
            ('/world/waves/model/rs750/link/base_link/sensor/anemometer/anemometer', '/wind/apparent'),
            ('/world/waves/model/rs750/link/base_link/sensor/imu_sensor/imu', '/rs750/IMU'),
            ('/world/waves/model/rs750/link/base_link/sensor/navsat_sensor/navsat', '/rs750/navsat')
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]   
    )   
    rudder = Node(
       package='rs750_ros',
       executable='twist_to_rudder.py',
       name= 'twist_to_rudder',
       output='both'
    )

    sail = Node(
       package='rs750_ros',
       executable='sail_control.py',
       name= 'sail_control',
       output='both'
    )

    heading_control = Node(
       package='rs750_ros',
       executable='heading_control.py',
       name= 'heading_control',
       output='both'
    )

    pose = Node(
       package='rs750_ros',
       executable='pose_generator.py',
       name= 'pose_generator',
       output='both'
    )

    battery = Node(
       package='rs750_ros',
       executable='ges_battery.py',
       name= 'ges_battery',
       output='both'
    )

    return LaunchDescription([
        bridge, rudder, sail, robot_state_publisher, heading_control, pose, battery,
            # Launch Arguments
            # DeclareLaunchArgument(
            #     'world',
            #     default_value='sailing_course_ges',
            #     description='Name of world'),
        gz_sim
    ])