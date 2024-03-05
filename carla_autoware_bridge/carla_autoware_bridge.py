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

from carla_autoware_bridge.aw_bridge import AutowareBridge
import rclpy


def main(args=None):
    rclpy.init(args=args)
    carla_autoware_bridge = AutowareBridge()

    rclpy.spin(carla_autoware_bridge)

    carla_autoware_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
