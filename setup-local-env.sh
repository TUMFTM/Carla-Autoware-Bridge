# ./src/Carla-Autoware-Bridge

sudo apt-get update \
  && sudo apt install software-properties-common -y \
  && sudo add-apt-repository ppa:deadsnakes/ppa -y \
  && sudo apt-get update \
  && sudo apt install python3-pip -y\
  && sudo apt install git curl -y\
  && sudo apt install python3-rosdep -y\
  && sudo apt install ros-humble-rmw-cyclonedds-cpp -y \
  && pip install -U colcon-common-extensions  \
  && curl https://syncandshare.lrz.de/dl/fiGEM2D9CSnoGfSNkJcMsH/carla-0.9.15-cp310-cp310-linux_x86_64.whl > carla-0.9.15-cp310-cp310-linux_x86_64.whl \
  && pip install carla-0.9.15-cp310-cp310-linux_x86_64.whl \
  && pip install simple-pid==1.0.1 \
  && pip install vcstool \
  && vcs import --recursive src < src/Carla-Autoware-Bridge/docker/docker.repos \
  && rosdep init && rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

source /opt/ros/$ROS_DISTRO/setup.bash && \
colcon build --symlink-install --packages-skip-build-finished --packages-up-to carla_autoware_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release

source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "$@"
