![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

# CaBot

CaBot (Carry on Robot) is an AI suitcase to help people with visual impairments travel independently. Can you imagine walking around at airports without vision? It's a huge open space, and there are lots of things and people so that it is hazardous for them to walk around at airports. [see project detail](https://www.cs.cmu.edu/~NavCog/cabot.html)

## CaBot v2

CaBot v2 has been migrated to ROS2 (humble). It uses Docker containers to maintain development/production systems.

### Hardware (example)
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
  - [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino/tree/ros2) for controlling handle, IMU, and other sensors
- GNSS Receiver (optional)
  - GNSS board with ublox ZED-F9P module
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
  - Host Ubuntu 20.04
  - See [jetson](doc/jetson.md) for detail

## Setup
- import third-party repos by using vcstool
  ```
  pip3 install vcstool # if you don't have vcs
  tools/setup-thirdparty-repos.sh
  ```
- run all scripts in tools based on your requirements
  ```
  cd tools
  ./install-service.sh               # need to install to configure system settings
  ./install-docker.sh                # if you need docker
  ./install-arm-emulator.sh          # if you build docker image for Jetson
  ./install-host-ros2.sh             # if you watch system performance or debug
  ./install-realsense-udev-rules.sh  # if you use realsense camera
  ./setup-display.sh                 # for display connections from docker containers
  ./setup-usb.sh                     # if you run physical robot
  ./setup-model.sh                   # if you need to recognize people
  ```

## Prepare Docker Images

### Pulling from dockerhub
- pulling docker containers
  ```
  ./manage-docker-image.sh -a pull -i "ros2 localization people people-nuc ble_scan" -o cmucal -t ros2-latest
  ```
- build workspace only
  ```
  ./build-docker.sh -w
  ```

### Build Docker Images
- build docker containers (at top directory)
  ```
  ./prebuild-docker.sh
  ./build-docker.sh
  ```

## Launch
- Run containers. Please configure the `.env` file before launching
  ```
  ./launch.sh          # for robot
  ./launch.sh -s       # for simulator
  ./launch.sh -c nuc   # for robot with nuc machine (need to configure CABOT_JETSON_CONFIG)
  ./launch.sh -c rs3   # for robot with 3 realsense configuration

  other options
    -d          # do not record ros2 bag
    -r          # record camera compressed images
    -n          # change log directory prefix (default=cabot)
    -v          # verbose
    -c <name>   # config name (default=) docker-compose(-<name>)(-production).yaml will use
                # if there is no nvidia-smi and config name is not set, automatically set to 'nuc'
    -3          # equivalent to '-c rs3'
  ```
- (optional) Run the gnss container before running launch.sh if you use a gnss receiver
  ```
  docker-compose -f docker-compose-gnss.yaml up
  ```


### .env file
- **Required settings**
  ```
  CABOT_MODEL          # robot model (default=) to determine which launch/urdf to use
  CABOT_SITE           # package name for cabot site (default=)
  CABOT_TOUCH_PARAMS   # touch sensor parameter for cabot-arduino handle (default=[128,48,24])
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # need to use cyclone dds due to performance issue
  ```
- Required settings for 3 Realsense configuration
  - `_X` should be replaced with `_1`, `_2`, or `_3` for each realsense
  ```
  CABOT_REALSENSE_SERIAL_X      # serial number of realsense
  CABOT_CAMERA_NAME_X           # camera name and camera should be at '<name>_link' (TF)
  ```
- Settings for the configuration using Jetson (experimental)
  - This will up people docker container on each specified jetson (by IP address or hostname).
  - Each jetson should connect to a Realsense
  - Each jetson should be ssh identification login enabled (without password) from the main machine
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
- Optional settings for ./launch.sh options in service
  ```
  CABOT_LAUNCH_CONFIG_NAME    # "", "nuc", "rs3"
  CABOT_LAUNCH_DO_NOT_RECORD  # 1/0
  CABOT_LAUNCH_RECORD_CAMERA  # 1/0
  CABOT_LAUNCH_LOG_PREFIX     # string, default=cabot
  ```
- Optional settings
  ```
  CABOT_BLE_VERSION    # BLE version setting (default=1)
                       # 0: do not use BLE, 1: use BLE server on ROS, 2: use BLE server outside of ROS (needs to setup)
  CABOT_LANG           # cabot language (default=en)
  CABOT_OFFSET         # offset size (default=0.25)
  CABOT_FOOTPRINT_RADIUS # robot radius size (default=0.45)
  CABOT_INIT_SPEED     # specify maximum robot speed at startup, leave empty to restore the last speed
  CABOT_MAX_SPEED      # you can change max_speed only when CABOT_MODEL is cabot2-gtm (defalt=1.0)
  CABOT_USE_GNSS       # to use GNSS fix for localization (default=0)
  CABOT_ANNOUNCE_NO_TOUCH # announce when the reason robot is stopped is NO_TOUCH(default=false)
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
  CABOT_SHOW_ROBOT_MONITOR   # show robot monitor (default=1)
  CABOT_RECORD_ROSBAG2       # record BT log, controller critics evalation into rosbag2 (default=1)
  CABOT_CAMERA_RGB_FPS       # camera RGB fps (default=30)
  CABOT_CAMERA_DEPTH_FPS     # camera depth fps (default=15)
  CABOT_CAMERA_RESOLUTION    # camera horizontal resolution (default=1280)
                             # need to use 848 or 640 if you use 3 realsense on a PC
  CABOT_DETECT_VERSION       # 1-3 (default=3)
                             # 1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet"
  CABOT_USE_ROBOT_TTS        # use TTS service '/speak_robot' to let PC speaker speak (default=0)
                             # this function is not used now, but maybe used in some scenario
  TEXT_TO_SPEECH_APIKEY      # IBM Cloud Text to Speech Service's API key and URL
  TEXT_TO_SPEECH_URL         # these two variables are required if CABOT_USE_ROBOT_TTS is 1
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
  ## The following will be managed by docker-compose files
  ## be careful to set these variables in your .env file
  ##
  CABOT_GAZEBO             # 1: gazebo 0: real robot
  CABOT_TOUCH_ENABLED      # true: enabled - false: disabled
                             disabled for gazebo and enabled for real robot in docker-compose file
  CABOT_USE_REALSENSE      # to use realsense camera (default=0)
                             disabled for gazebo and enabled for real robot in docker-compose file
  CABOT_PRESSURE_AVAILABLE # to use pressure sensor (default=0}
                             disabled for gazebo and enabled for real robot in docker-compose file
  ```
- DDS related
  ```
  NET_CORE_BUFFER_SIZE_IN_MB   # default 64, small buffer size can cause data congestion and degrade performance especially velodyne_points and localization
  ```

### Navigate CaBot

- **`Nav2 Goal` tool does not work properly**: the robot will move with the nav2 default BT xml (only for debugging purposes)
- right-click on a blue dot in `demo_2d_floors.rviz` (-Y option for ros1 service to show) and select the "Navigate to Here" menu
- or directory publish a `/cabot/event` topic on ROS1. see [here](doc/destinations.md) more detail about destinations.
  ```
  # example destination in cabot_site_cmu_3d environment
  $ rostopic pub -1 /cabot/event std_msgs/String "data: 'navigation;destination;EDITOR_node_1496171299873'"
  ```

### CaBot app for iOS

TBD

## Customization (build your own map)

See [customization](doc/customization.md) for more details.

## Development Detail
See [development](doc/development.md) for more details.

## Getting Involved

### Issues and Questions

Please use Issues for both issue tracking and your questions about the CaBot repository.

### Developer Certificate of Origin (DCO)

The developer needs to add a Signed-off-by statement and thereby agrees to the DCO, which you can find below. You can add either -s or --sign-off to your usual git commit commands. If Signed-off-by is attached to the commit message, it is regarded as agreed to the Developer's Certificate of Origin 1.1.


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
