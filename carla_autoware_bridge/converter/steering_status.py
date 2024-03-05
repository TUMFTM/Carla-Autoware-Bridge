#   Copyright (c) 2023 Robotics010
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

from autoware_auto_vehicle_msgs.msg import SteeringReport
from carla_autoware_bridge.converter.converter import Converter
from carla_msgs.msg import CarlaEgoVehicleStatus
import numpy as np


class SteeringStatusConverter(Converter):

    def __init__(self) -> None:
        super().__init__()
        self._steer_to_angle_polynomial = self._calculate_steer_to_angle_polynomial()

    def _calculate_steer_to_angle_polynomial(self):
        max_left_steer = -1
        max_right_steer = 1

        fl_max_left_angle = -48.99
        fr_max_left_angle = -35.077
        average_max_left_angle = -(fl_max_left_angle + fr_max_left_angle) / 2
        average_max_left_angle = np.radians(average_max_left_angle)
        average_max_right_angle = -average_max_left_angle

        x = [max_left_steer, max_right_steer]
        y = [average_max_left_angle, average_max_right_angle]
        polynomial_coefficients = np.polyfit(x, y, 1)
        steer_to_angle_polynomial = np.poly1d(polynomial_coefficients)
        return steer_to_angle_polynomial

    def _convert(self):
        if not isinstance(self._inbox, CarlaEgoVehicleStatus):
            raise RuntimeError(f'Input must be {CarlaEgoVehicleStatus}!')

        output_steering_status = SteeringReport()
        steer = self._inbox.control.steer
        angle = self._steer_to_angle_polynomial(steer)
        output_steering_status.steering_tire_angle = angle
        self._outbox = output_steering_status
