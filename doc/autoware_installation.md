# Autoware Installation

## Create root folder
`AV20_ROOT` is the local folder where to clone autoware
    
    mkdir ~/av20
    cd ~/av20
    AV20_ROOT=$PWD

## Get AW
    git clone https://github.com/autowarefoundation/autoware.git
    cd autoware
    git checkout 2024.01

## Create AW container
    docker pull ghcr.io/autowarefoundation/autoware-universe:humble-2024.01-cuda-amd64
    rocker --network=host -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --nvidia --volume $AV20_ROOT/autoware -e AV20_ROOT -- ghcr.io/autowarefoundation/autoware-universe:humble-2024.01-cuda-amd64

## In AW container: Do adjustments and build
    cd AV20_ROOT/autoware
    mkdir src
    vcs import src < autoware.repos
    cd src
    git clone https://github.com/TUMFTM/Carla_t2.git
    cd ..
    # Do adjustments acc. to https://github.com/TUMFTM/Carla-Autoware-Bridge/blob/main/doc/autoware-changes.md
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

## Get map
From here: https://syncandshare.lrz.de/getlink/fiBgYSNkmsmRB28meoX3gZ/ and extract to `AV20_ROOT/Town10`

