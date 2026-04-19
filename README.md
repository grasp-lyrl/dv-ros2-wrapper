# DV ROS2

ROS2 drivers and sample nodes for iniVation cameras and DV software infrastructure.

## Installation on Ubuntu OS

The code depends on DV software libraries. Enable the iniVation PPA for your Ubuntu distribution:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install dv-processing dv-runtime-dev gcc-13 g++-13
```

Some extra ROS2 dependencies are also needed:

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions ros-humble-camera-info-manager ros-humble-tf2-sensor-msgs ros-dev-tools
```

Clone the repository and build:

```bash
mkdir -p ~/dv_ws/src && cd ~/dv_ws/src
git clone git@github.com:grasp-lyrl/neurofly.git
cd ~/dv_ws
rosdep install --from-paths src/neurofly/dv-ros2-wrapper --ignore-src -r -y
colcon build --symlink-install --parallel-workers 4 \
             --cmake-args -DCMAKE_BUILD_TYPE=Release \
                          -DCMAKE_C_COMPILER=gcc-13 \
                          -DCMAKE_CXX_COMPILER=g++-13
```

## Verifying the build

After the build, source your environment, connect your iniVation camera, and validate by running the capture driver:

```bash
source ~/dv_ws/install/local_setup.bash
ros2 launch dv_ros2_capture driver.launch.py
```

You should see events and IMU frames streaming from the connected iniVation camera.

## Compatibility

The message types for events are designed to be compatible with types available in
[rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros), enabling communication with nodes developed using those event
message types (`Event` and `EventArray`).

## Repository structure

The `dv-ros2-wrapper` directory inside this repository contains the following packages:

- `dv_ros2_msgs` - Basic message types for iniVation cameras (events, frames, IMU, triggers)
- `dv_ros2_messaging` - C++ headers for using dv-processing within ROS2 nodes
- `dv_ros2_capture` - Camera driver node supporting live streaming from iniVation cameras
- `dv_ros2_visualization` - Simple event stream visualization node
