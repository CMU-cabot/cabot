![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

# CaBot

CaBot (Carry on Robot) is an AI suitcase to help people with visually impairments travel independently. Can you imagine to walk around at airports without vision? It’s huge open space and there are lots of things and people, so that it is really dangerous for them to walk around at airports. [see project detail](https://www.cs.cmu.edu/~NavCog/cabot.html)

## CaBot v2

CaBot v2 uses ROS1, ROS2, and ros1_bridge to use [navigation2](https://github.com/ros-planning/navigation2) package for ROS2 and existing packges for ROS1. Also, it uses Docker container to maintain development/production systems.

### Hardware assumption
Item|Product
---|---
LiDAR|Velodyne VLP-16
Stereo Camera|RealSense D435
Motor Controller|ODrive Motor Controller v3.6 or above (Firmware v0.5.1 or above)
Micro Controller|Arduino [Link TBD]() for controlling handle, IMU, and other sensors
PC|ZOTAC Magnus EN72070V

### Localization
- AMCL or mf_localization (cartogrpher+iBeaons)

### Tested Environment

- Host Ubuntu 20.04
- Docker v20
- docker-compose v1.28~
- Docker containers
  - `ros1`: Ubuntu20.04, ROS1 noetic
  - `ros2`: Ubuntu20.04, ROS2 galactic
  - `bridge`: Ubuntu20.04, ROS1 noetic, ROS2 galactic
  - `localization`: Ubuntu20.04, ROS1 noetic
  - `people`: Ubuntu20.04, ROS1 noetic

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
./setup-display.sh                 # for display connections from docker containers
./setup-usb.sh                     # if you run physical robot
./setup-model.sh                   # if you need to recognize people
./install-realsense-udev-rules.sh  # if you use realsense camera
```
- build docker containers
```
cd docker
./prebuild-docker.sh [-p <project_name>]
./build-docker.sh [-p <project_name>] [-P]

-p option can specify docker-compose's -p option to build docker images in different name prefix
   Please check docker-compose help to see the detail.
-P option with build-docker.sh will also run ./prebuild-docker.sh
```
- prepare docker/.env file
  - ROS_IP       host machine IP address
  - MASTER_IP    ROS1 master IP address
- run containers. This will show up Rviz.
```
./launch.sh [-p <project_name>] [-n <log_name_prefix>] [-r] [-s]

-p option can specify docker-compose's -p option
-n set log name prefix (all logs and bag file should be stored in a directory) default=cabot
-r record camera image
-s launch in simulation mode
```

### Navigate CaBot on Gazebo simulation

- `Navigation 2 Goal` tool does not work properly. 
- You need to use CaBot menu instead, find `xterm` terminal displaying `type 'j', 'k', or 'l' for 'up', 'center', 'down' buttons`
- Type `k`, `k`, `k` to start navigation.

### CaBot app for iOS

TBD

## Customization

See [customization](doc/customization.md) for more details.

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
