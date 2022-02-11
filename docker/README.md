# Docker configuration for CaBot

CaBot utilizes docker containers to run and manage various kinds of packages.

## Files

- home directory will be mounted by the containers (see docker-compose files).
- `../tools/setup-thirdparty-repos.sh`  will import thirdparty packages into mostly under workspaces or docker contexts

```
- home/
  - .ros/         - ROS home
    - log/        - ROS log files
  - bridge_ws     - workspace for bridge
  - catkin_ws     - workspace for ros1
  - loc_ws        - workspace for localization
  - people_ws     - workspace for people
  - people_nuc_ws - workspace for people (without people detection)
  - ros2_ws       - workspace for ros2
- bridge          - bridge docker context
- localization    - localization docker context
- people          - people docker context
- ros1            - ros1 docker context
- ros2            - ros2 docker context
- prebuild        - docker contexts for base images
```

## Main Docker Images

There are docker image context directory for the following five images

|Image       |From                           |Additional Layers |ROS functions|
|------------|-------------------------------|------------------|-----------|
|ros1        |focal-noetic-base-mesa         |ROS1 noetic       |UI, BLE, device controller|
|ros2        |focal-galactic-desktop-nav2-mesa|debug tools       |Nav2       |
|people      |focal-cuda11.1-cudnn8-devel-noetic-base|OpenCV, Open3D, Realsense|people detection and tracking, queue detector|
|people-nuc  |focal-noetic-base-mesa         |dependencies      |queue detector, people tracking|
|localization|focal-noetic-base-mesa         |Cartographer dependency|localization, RF signal scan|
|bridge      |focal-galactic-desktop-nav2-mesa|ROS1 noetic|ROS1-2 bridge|

### Utility Images
|Image       |From                           |Additional Layers |ROS functions|
|------------|-------------------------------|------------------|-----------|
|topic_checker|localization|N/A|utility|
|ble_scan/wifi_scan|focal-noetic-base-mesa|dependency|Bluetooth/Wi-Fi scanning|

### Image for Jetson

people context has a Dockerfile for jeston named `Dockerfile.jetson`

### Image for NUC

people context has a Dockerfile for PC without nvidia GPU named `Dockerfile.nuc`, which is for running people/queue packages excluding people detection.

## Prebuild Docker Images

In prebuild directory, there are some docker image context for base images for main images.

|Image|From|Additional Layers|
|---|---|---|
|focal-noetic-base-mesa|focal-noetic-base|Mesa utils|
|focal-noetic-base|focal-noetic|ROS1 noetic base|
|focal-noetic|ubuntu:focal|ROS1 noetic core|
|focal-galactic-desktop-nav2-mesa|focal-galactic-desktop-nav2|Mesa utils|
|focal-galactic-desktop-nav2|focal-galactic-desktop|Navigation 2|
|focal-galactic-desktop|ros:galactic|ROS2 galactic desktop|
|focal-cuda11.1-cudnn8-devel-noetic-base|focal-cuda11.1-cudnn8-devel-noetic|ROS1 noetic base|
|focal-cuda11.1-cudnn8-devel-noetic|nvidia/cuda:11.1-cudnn8-devel-ubuntu20.04|ROS1 noetic core|
|nvidia/cuda:11.1-cudnn8-devel-ubuntu20.04|dockerhub||
|ros:galactic|dockerhub||
|ubuntu:focal|dockerhub||


## ROS1 Bridge (communication between ROS1 and ROS2)

- customized [ros1_bridge](https://github.com/daisukes/ros1_bridge/tree/enhance-parameter-bridge) is used to control latched topics
  - `./home/bridge_ws/bridge_topics_sim.yaml` is bridging meesage configuration
- patched [action_bridge](https://github.com/daisukes/action_bridge/tree/fix-galactic-temp) is used for actions
- [nav2_action_bridge](../nav2_action_bridge) is implementation of action bridge