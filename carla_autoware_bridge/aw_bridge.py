# Copyright 2023 Gemb Kaljavesi, Technical University of Munich
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from ackermann_msgs.msg import AckermannDrive 
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport
from carla_autoware_bridge.converter.steering_status import SteeringStatusConverter
from carla_autoware_bridge.converter.velocity_report import VelocityReportConverter
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy import qos, Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped
import copy

import numpy as np


class AutowareBridge(Node):

    def __init__(self) -> None:
        super().__init__('carla_autoware_bridge')
        self._velocity_report_converter = VelocityReportConverter()
        self._steering_status_converter = SteeringStatusConverter()

        # The vehicle status/ steering status is fixed and cannot be set
        self._vehicle_status_subscriber = self.create_subscription(
            CarlaEgoVehicleStatus, '/carla/ego_vehicle/vehicle_status',
            self._vehicle_status_callback, qos.qos_profile_sensor_data)
        self._steering_status_publisher = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', 1)
        
        self._odometry_subscriber = self.create_subscription(
            Odometry, '/carla/ego_vehicle/odometry',
            self._odometry_callback, qos.qos_profile_sensor_data)
        self._velocity_report_publisher = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', 1)
        # Send directly pose_with_covariance
        # New gnss poser does not support the right conversion anymore
        self._pose_with_cov_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', 1)
        
        # Subscribe autoware ackermann control cmd and send to the PID
        self._ackermann_control_command_subscriber = self.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd',
            self._control_callback, qos.qos_profile_sensor_data)
        self._ackermann_pub = self.create_publisher(
            AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 1)
       
    def _odometry_callback(self, odometry_msg):
        # velocity msg
        self._velocity_report_converter.inbox = odometry_msg
        self._velocity_report_converter.convert()
        velocity_report_msg = self._velocity_report_converter.outbox

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = odometry_msg.header.stamp
        velocity_report_msg.header = header
        self._velocity_report_publisher.publish(velocity_report_msg)

        # pose_with_cov msg
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header.frame_id = 'map'
        out_pose_with_cov.header.stamp = odometry_msg.header.stamp
        out_pose_with_cov.pose.pose = odometry_msg.pose.pose
        out_pose_with_cov.pose.covariance = [
            0.1,0.0,0.0,0.0,0.0,0.0,
            0.0,0.1,0.0,0.0,0.0,0.0,
            0.0,0.0,0.1,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0
            ]
        self._pose_with_cov_publisher.publish(out_pose_with_cov)

    def _vehicle_status_callback(self, vehicle_status_msg):
        self._steering_status_converter.inbox = vehicle_status_msg
        self._steering_status_converter.convert()

        steering_status_msg = self._steering_status_converter.outbox
        steering_status_msg.stamp = self.get_clock().now().to_msg()
        self._steering_status_publisher.publish(steering_status_msg)

    def _control_callback(self, aw_ackermann_control_command_msg):
        carla_ackermann_control = AckermannDrive()
        carla_ackermann_control.steering_angle = aw_ackermann_control_command_msg.lateral.steering_tire_angle * 1.2
        carla_ackermann_control.steering_angle_velocity = aw_ackermann_control_command_msg.lateral.steering_tire_rotation_rate * 1.2
        carla_ackermann_control.speed = aw_ackermann_control_command_msg.longitudinal.speed
        carla_ackermann_control.acceleration = aw_ackermann_control_command_msg.longitudinal.acceleration
        carla_ackermann_control.jerk = aw_ackermann_control_command_msg.longitudinal.jerk
        self._ackermann_pub.publish(carla_ackermann_control)

