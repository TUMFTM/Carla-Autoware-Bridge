#   Copyright (c) 2023 Gemb Kaljavesi, Robotics010
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

from autoware_vehicle_msgs.msg import VelocityReport
from carla_autoware_bridge.converter.converter import Converter
from nav_msgs.msg import Odometry


class VelocityReportConverter(Converter):

    def _convert(self):
        if not isinstance(self._inbox, Odometry):
            raise RuntimeError(f'Input must be {Odometry}!')

        # Convert from left-handed Unreal coordinate frame
        # to right-handed ROS2 coordinate frame
        yaw_rate = self._inbox.twist.twist.angular.z
        longitudinal_velocity = self._inbox.twist.twist.linear.x
        lateral_velocity = self._inbox.twist.twist.linear.y

        output_velocity_report = VelocityReport()
        output_velocity_report.heading_rate = -yaw_rate
        output_velocity_report.longitudinal_velocity = longitudinal_velocity
        output_velocity_report.lateral_velocity = -lateral_velocity
        self._outbox = output_velocity_report
