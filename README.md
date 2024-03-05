<div align="center">

# CARLA-Autoware-Bridge - Enables the use of CARLA for Testing and Development of Autoware Core/Universe
[![Linux](https://img.shields.io/badge/os-ubuntu22.04-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

</div>

## Introduction
The CARLA-Autoware-Bridge is a package to connect the CARLA simulator to Autoware Core/Universe with the help of the CARLA-ROS-Bridge. Currently the **latest Autoware Core/Universe** and **CARLA 0.9.15** is supported.

## Paper
If you use this or the other associated repos, please cite our Paper:

**CARLA-Autoware-Bridge: Facilitating Autonomous Driving Research
with a Unified Framework for Simulation and Module Development**<br>Gemb Kaljavesi, Tobias Kerbl, Tobias Betz, Kirill Mitkovskii, Frank Diermeyer [[PDF]()]

```
@inproceedings{carla_aw_bridge24,
  title = {CARLA-Autoware-Bridge: Facilitating Autonomous Driving Research
with a Unified Framework for Simulation and Module Development,
  author = {Kaljavesi, Gemb and Kerbl, Tobias and Betz, Tobias and Mitkovskii, Kirill and Diermeyer, Frank},
  year = {2024}
}
```

The Paper is currently under review and only published as preprint.

## Overview
The simulation framework around the CARLA-Autoware-Bridge consists of the components:
- carla-autoware-bridge: This repository holding the CARLA-Autoware-Bridge.
- ros-bridge: Fork of the ros-bridge with our changes needed for the CARLA-Autoware-Bridge.
- carla-t2: Vehicle model and sensor kit packages of the CARLA T2 2021 Vehicle for Autoware.
- carla-ros-msgs:  Fork of the carla-ros-msg with our changes needed for the CARLA-Autoware-Bridge.

## How to Build and Install the Bridge
The easiest way to use the CARLA-Autoware-Bridge is to use our prebuilt docker image or to build the docker image by yourself. Bu we also provide a tutorial for local usage.

#### Docker Workflow(Recommended)
You can build the docker image by yourself or use the image from our github registry.
```
# Pull our latest docker image
docker pull tumgeka/carla-autoware-bridge:latest

# Alternatively build it yourself by running our build_docker.sh
./docker/build_docker.sh
```

#### Local Workflow
Comming Soon. Until then, take a look at our Dockerfile.

#### Maps
Autoware needs the maps in a special lanelet2 format, we will upload all converted maps in the future under the following link: [carla-autoware-bridge/maps](https://syncandshare.lrz.de/getlink/fiCzVPnWWS4eGXLK8MwZNy/)

## General Installation and Usage
The installation and usage of the CARLA-Autoware-Bridge is described in the following tutourial. In order to function properly the packages should be started in the order CARLA --> CARLA-Autoware-Bridge --> Autoware. 

### 1) CARLA
We recommended to use the dockerized version of CARLA 0.9.15. To pull and start CARLA for usage with the CARLA-Autoware-Bridge follow the steps below.
```bash
# Pull CARLA 0.9.15
docker pull carlasim/carla:0.9.15
```
```bash
# Start CARLA
docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.15 /bin/bash ./CarlaUE4.sh -carla-rpc-port=1403 
```
Additional information:
- `-prefernvidia` - use NVIDIA GPU for hardware acceleration
- `-carla-rpc-port=3000` - use other than default port (2000) for RPC service's port
- `-quality-level=Low` - use low quality level mode for a minimal video memory consumption

### 2) CARLA-Autoware-Bridge
Run the carla-autoware-bridge
```bash
# If you are using a docker start the docker first
docker run -it -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --network host tumgeka/carla-autoware-bridge:latest

# Launch the bridge
ros2 launch carla_autoware_bridge carla_aw_bridge.launch.py port:=1403 town:=Town10HD
```

Additional information:
- `-port:=3000` to switch to a different CARLA port for it's RPC port
- `-timeout:=10` to increase waiting time of loading a CARLA town before raising error
- `-view:=true` to show third-person-view window
- `-town:=Town10HD` to set the town
- `-traffic_manager=False` to turn off traffic manager server (True by default)
- `-tm_port=8000` to switch the traffic manager server port to a different one (8000 by default)

### 3) Autoware
To use Autoware some minor [adjustments](/doc/autoware-changes.md) are required. Additionally you will need our sensorkit and vehicle model.
```
git clone https://github.com/TUMFTM/Carla_t2.git
```

Launch autoware
```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=carla_t2_vehicle sensor_model:=carla_t2_sensor_kit map_path:=<path to /wsp/map>
```

Autoware changes often, for a reproducible behaviour we recommend you to use a tagged autoware version:
https://github.com/autowarefoundation/autoware/tree/v1.0

```bash
docker pull ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda

rocker --network=host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --nvidia --volume /path/to/code -- ghcr.io/autowarefoundation/autoware-universe:humble-1.0-cuda
```