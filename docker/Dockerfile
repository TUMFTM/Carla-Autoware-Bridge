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

FROM ros:humble-ros-core-jammy

COPY . /tum/src/carla_autoware_bridge
WORKDIR /tum

# Install apt packages
RUN apt-get update && apt install software-properties-common -y \
  && add-apt-repository ppa:deadsnakes/ppa -y \
  && apt-get update \
  && apt install python3-pip -y\
  && apt install git curl -y\
  && apt install python3-rosdep -y\
  && apt install ros-humble-rmw-cyclonedds-cpp -y \
  && pip install -U colcon-common-extensions  \
  && DEBIAN_FRONTEND=noninteractive \
  && curl https://syncandshare.lrz.de/dl/fiGEM2D9CSnoGfSNkJcMsH/carla-0.9.15-cp310-cp310-linux_x86_64.whl > carla-0.9.15-cp310-cp310-linux_x86_64.whl \
  && pip install carla-0.9.15-cp310-cp310-linux_x86_64.whl \
  && pip install simple-pid==1.0.1 \
  && pip install vcstool \
  && vcs import --recursive src < src/carla_autoware_bridge/docker/docker.repos \
  && rosdep init && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Build packages
RUN  bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
  colcon build --symlink-install --packages-skip-build-finished --packages-up-to carla_autoware_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Create entrypoint
RUN echo '#!/usr/bin/env bash' > /ros_entrypoint.sh
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /ros_entrypoint.sh
RUN echo 'source /tum/install/setup.bash' >> /ros_entrypoint.sh
RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /ros_entrypoint.sh
RUN echo 'exec "$@"' >> /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


  
