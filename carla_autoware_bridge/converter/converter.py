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

from abc import ABC, abstractmethod


class Converter(ABC):

    def __init__(self, default_inbox=None, default_outbox=None) -> None:
        super().__init__()
        self._inbox = default_inbox
        self._outbox = default_outbox

    @property
    def inbox(self):
        return self._inbox

    @inbox.setter
    def inbox(self, value):
        self._inbox = value

    @property
    def outbox(self):
        if self._outbox is None:
            raise RuntimeError('Call convert before getting outbox!')
        return self._outbox

    def convert(self):
        if self._inbox is None:
            raise RuntimeError('Set inbox before calling convert!')
        self._convert()

    @abstractmethod
    def _convert(self):
        pass
