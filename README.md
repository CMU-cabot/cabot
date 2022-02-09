![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

# CaBot

CaBot (Carry on Robot) is an AI suitcase to help people with visually impairments travel independently. Can you imagine to walk around at airports without vision? It’s huge open space and there are lots of things and people, so that it is really dangerous for them to walk around at airports. [see project detail](https://www.cs.cmu.edu/~NavCog/cabot.html)

## CaBot v2

CaBot v2 uses ROS1, ROS2, and ros1_bridge to use [navigation2](https://github.com/ros-planning/navigation2) package for ROS2 and existing packges for ROS1. Also, it uses Docker container to maintain development/production systems.

### Hardware
- Robot frame + handle
  - [CaBot2-E2 model](https://github.com/CMU-cabot/cabot_design/tree/master/cabot2-e2)
  - [CaBot2-GT model](https://github.com/CMU-cabot/cabot_design/tree/master/cabot2-gt)
- LiDAR
  - Velodyne VLP-16
- Stereo Camera(s)
  - 1 RealSense camera (D435)
  - 2~ RealSense cameras (with a PC or NUC + Jetson cluster)
- Motor Controller
  - ODrive Motor Controller v3.6 (Firmware v0.5.1)
- Micro Controller (Handle and sensors)
  - [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino) for controlling handle, IMU, and other sensors
- Processor
  - PC with NVIDIA GPU (ZOTAC Magnus EN72070V)
  - NUC (Ruby R8) + Jetson Mate (multiple Jetson Xavier NX)

### Localization
- [mf_localization](https://github.com/CMU-cabot/cabot/tree/dev/mf_localization) (cartogrpher+iBeacons/WiFi)

### Tested Environment
- PC
  - Host Ubuntu 20.04
  - Docker v20
  - docker-compose v1.28~v1.29.2
- Jetson
  - Host Ubuntu 18.04
  - See [jetson](doc/jetson.md) for detail

## Setup
- import thirdparty repos by using vcstool
  ```
  pip3 install vcstool # if you don't have vcs
  tools/setup-thirdparty-repos.sh
  ```
- run all script in tools based on your requirements
  ```
  cd tools
  ./install-docker.sh                # if you need docker
  ./install-arm-emulator.sh          # if you build docker image for Jetson
  ./install-host-ros.sh              # if you watch system performance
  ./install-realsense-udev-rules.sh  # if you use realsense camera
  ./setup-display.sh                 # for display connections from docker containers
  ./setup-usb.sh                     # if you run physical robot
  ./setup-model.sh                   # if you need to recognize people
  ```

## Prepare Docker Images

### Pulling from dockerhub
- pulling docker containers
  ```
  ./manage-docker-image.sh -a pull -i all -o cmucal
  ```
- build workspace only
  ```
  ./build-docker.sh -w
  ```

### Build Docker Images
- build docker containers (at top direcotry)
  ```
  ./build-docker.sh -P                              # for build all images for PC with nVIDIA gpu
  ./build-docker.sh -g mesa -P                      # for build all images for PC with mesa/OpenGL compatible gpu
  ```

## Launch
- Run containers. Please configure `.env` file before launching
  ```
  ./launch.sh          # for robot
  ./launch.sh -s       # for simulator
  ./launch.sh -c nuc   # for robot with nuc machine (need to configure CABOT_JETSON_CONFIG)
  ./launch.sh -c rs3   # for robot with 3 realsense configuration

  other options
    -d          # do not record rosbag (ROS1)
    -r          # record camera compressed images
    -n          # change log directory prefix (default=cabot)
    -v          # verbose
    -c <name>   # config name (default=) docker-compose(-<name>)(-production).yaml will use
                # if there is no nvidia-smi and config name is not set, automatically set to 'nuc'
    -3          # equivalent to '-c rs3'
  ```

### .env file
- Basic configuration
  ```
  ROS_IP               # host machine IP address or 127.0.0.1 for single PC setting (default=)
  MASTER_IP            # ROS1 master's IP address or 127.0.0.1 for single PC setting (default=)
  CABOT_MODEL          # robot model (default=)
  CABOT_NAME           # robot name (default=)
  CABOT_SITE           # package name for cabot site (default=)
  CABOT_LANG           # cabot language (default=en)
  CABOT_OFFSET         # offset size (default=0.25)
  CABOT_TOUCH_PARAMS   # touch sensor parameter for cabot-arduino handle (default=[128,48,24])
  CABOT_INIT_SPEED     # specify robot maximum speed at start up, leave empty to restore the last speed
  ```
- Options for multiple jetson/realsense configuration.
  - This will up people docker container on each specified jetson (by IP address or host name).
  - Each jetson should connect to a Realsense
  - Each jeston should be ssh identification login enabled (without password) from the main machine
  - Each jetson's `.env` file should be configured proper `ROS_IP` and `MASTER_IP` setting
  ```
  CABOT_JETSON_USER    # User name to login jetson (default=cabot)
  CABOT_JETSON_CONFIG  # Space separated config for muliple jeston/realsense
    #
    # "<<Mode>:<HOST>:<Name>[ <Mode>:<IP Address>:<Name>]*"
    # Mode: D - detection
    #     : T - tracking (only for testing, the PC will launch tracking processes)
    # HOST: IP address or host name of the jeston
    # Name: Used for camera name space "<Name>" and camera TF link "<Name>_link"
    #
    # ex) "D:192.168.1.50:rs1 D:192.168.1.51:rs2"
  ```
- Options for 3 Realsense configuration.
  - `_X` should be replaced with `_1`, `_2`, or `_3` for each realsense
  ```
  CABOT_REALSENSE_SERIAL_X      # serial number of realsense
  CABOT_CAMERA_NAME_X           # camera name and camera should be at '<name>_link' (TF)
  ```

- Options for debug/test
  ```
  CABOT_GAMEPAD              # (default=gamepad) gamepad type for remote controll (ex. PS4 controller)
                                               pro (Nintendo Switch Pro controller)
  CABOT_SHOW_GAZEBO_CLIENT   # show gazebo client (default=0)
  CABOT_SHOW_ROS1_RVIZ       # show ROS1 rviz (default=0)
  CABOT_SHOW_ROS2_RVIZ       # show ROS2 rviz (default=1)
  CABOT_SHOW_ROS2_LOCAL_RVIZ # show ROS2 local navigation rviz (default=0)
  CABOT_SHOW_LOC_RVIZ        # show ROS1 localization rviz (default=1)
  CABOT_SHOW_PEOPLE_RVIZ     # show ROS1 people rviz (default=0)
  CABOT_RECORD_ROSBAG2       # record BT log, controller critics evalation into rosbag2 (default=1)
  CABOT_DETECT_VERSION       # 0-3 (default=3)
                             # 0: python-darknet, 1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet"
  ```
- Options for simulation
  ```
  CABOT_INITX          # initial robot position x for gazebo (default=0)
  CABOT_INITY          # initial robot position y for gazebo (default=0)
  CABOT_INITZ          # initial robot position z for gazebo (default=0)
  CABOT_INITA          # initial robot angle (degree) for gazebo (default=0)
  ```
- Others
  ```
  ## the following will be managed by docker-compose files
  ## be careful to set these variables in your .env file
  ##
  CABOT_GAZEBO             # 1: gazebo 0: real robot
  CABOT_TOUCH_ENABLED      # to enable touch speed control (default=1)
                             disabled for gazebo and enabled for real robot in docker-compose file
  CABOT_USE_REALSENSE      # to use realsense camera (default=0)
                             disabled for gazebo and enabled for real robot in docker-compose file
  CABOT_PRESSURE_AVAILABLE # to use pressure sensor (default=0}
                             disabled for gazebo and enabled for real robot in docker-compose file
  ```

### Navigate CaBot

- **`Nav2 Goal` tool does not work properly**: the robot will move with the nav2 default BT xml (only for debugging purpose)
- right click on a blue dot in `demo_2d_floors.rviz` (-Y option for ros1 service to show) and select "Navigate to Here" menu
- or directory publish a `/cabot/event` topic on ROS1. see [here](doc/destinations.md) more detail about destinations.
  ```
  # example destination in cabot_site_cmu_3d environment
  $ rostopic pub -1 /cabot/event std_msgs/String "data: 'navigation;destination;EDITOR_node_1496171299873'"
  ```

### CaBot app for iOS

TBD

## Customization

See [customization](doc/customization.md) for more details.

## Development Detail
See [development](doc/development.md) for more details.

## Getting Involved

### Issues and Questions

Please use Issues for both issue tracking and your questions about CaBot repository.

### Developer Certificate of Origin (DCO)

The developer need to add a Signed-off-by statement and thereby agrees to the DCO, which you can find below. You can add either -s or --signoff to your usual git commit commands. If Signed-off-by is attached to the commit message, it is regarded as agreed to the Developer's Certificate of Origin 1.1.


https://developercertificate.org/
```
Developer's Certificate of Origin 1.1

By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the open source license
    indicated in the file; or

(b) The contribution is based upon previous work that, to the
    best of my knowledge, is covered under an appropriate open
    source license and I have the right under that license to
    submit that work with modifications, whether created in whole
    or in part by me, under the same open source license (unless
    I am permitted to submit under a different license), as
    Indicated in the file; or

(c) The contribution was provided directly to me by some other
    person who certified (a), (b) or (c) and I have not modified
    it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including
    all personal information I submit with it, including my
    sign-off) is maintained indefinitely and may be redistributed
    consistent with this project or the open source license(s)
    involved.
```

# License

[MIT License](LICENSE)


---
The following files/folder are under Apache-2.0 License

- cabot_description/urdf/sensors/_d435.gazebo.xacro
- cabot_description/urdf/sensors/_d435.urdf.xacro
- cabot_navigation/launch/cartographer_mapping.launch
- nav2_action_bridge/cmake/find_ros1_package.cmake
- docker/bridge/ros1/nav2_msgs/
