# Docker configuration for CaBot

CaBot utilizes docker containers to run and manage various kinds of packages.

## Files

- the containers will mount the home directory (see docker-compose files).
- `../tools/setup-thirdparty-repos.sh`  will import third party packages into mostly under workspaces or docker contexts

```
- home/
  - .ros/         - ROS home
    - log/        - ROS log files
  - loc_ws        - workspace for localization
  - people_ws     - workspace for people
  - people_nuc_ws - workspace for people (without people detection)
  - ros2_ws       - workspace for ros2
- localization    - localization docker context
- people          - people docker context
- prebuild        - docker contexts for base images
- ros2            - ros2 docker context
- server          - docker context for local map service server
- timezone        - for adjusting timezone of the docker image
- uid             - for adjusting docker image uid
- vscode          - for development
```

## Main Docker Images

There are docker image context directory for the following five images

|Image       |From              |Additional Layers |ROS functions|
|------------|------------------|------------------|-----------|
|ros2        |jummy-humble      |debug tools       |Nav2       |
|people      |jammy-cuda11.7.1  |OpenCV, Open3D, Realsense|people detection and tracking, queue detector|
|people-nuc  |jummy-humble      |dependencies      |queue detector, people tracking|
|localization|jummy-humble      |Cartographer      |localization, RF signal scan|
|rtk-gnss    |jummy-humble      |GNSS              |outdoor localization|
|server      |ubuntu:focal      |Open Liberty      |map service server|

### Utility Images
|Image       |From                           |Additional Layers |ROS functions|
|------------|-------------------------------|------------------|-----------|
|ble_scan/wifi_scan|focal-noetic-base-mesa|dependency|Bluetooth/Wi-Fi scanning|

### Image for Jetson

people context has a Dockerfile for jeston named `Dockerfile.jetson`

### Image for NUC

People context has a Dockerfile for PC without NVIDIA GPU named `Dockerfile.nuc`, which runs people/queue packages excluding people detection.

## Prebuild Docker Images

In prebuild directory, there are some docker image contexts for base images for main images.

|Image|Note|
|---|---|
|_jammy-humble-*|Base image for the docker without GPU process|
|_jammy-cuda11.7.1-*|Base image for the docker with GPU process|

###

|Prebuild|Note|
|cv|OpenCV and Open3D|
|docker_images|for humble|
|humble-custom|additional packages for humble|
|jetson-humble-base-src|additional packages for humble on Jetson|
|mesa|packages for display|
|opengl|packages for display|
|vcs|install ros2 build tools|
