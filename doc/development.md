## Development details

## Scripts

build docker images 

- see `./prebuild-docker.sh -h` for options
- see `./build-docker.sh -h` for options

### Manage docker images script

A utility script to tag, pull, push, list, or rmi (remove) images.
(not support l4t, jetson image yet, see [jetson](jetson.md))

- see `./manage-docker-image.sh -h` for options

### Docker-compose files

[see docker](../docker) for details of each images.
Docker images manage required software/library to build and run CaBot packages.
Most of CaBot packages will not be built as docker image layers.

So, most ROS1/2 CaBot workspaces should be built after building/pulling docker images.
docker-compose files mount home directory (`docker/home`) and required CaBot packages into corresponding workspace to keep built binaries on the host file systems.

`launch.sh` will select proper docker-compose file to launch.

|File|Real/Simulation|Processor|Realsense|Description|
|---|---|---|---|---|
|docker-compose-common.yaml|-|x86_64|0/1|Common declaration|
|docker-compose.yaml|simulation|x86_64 + NVIDIA GPU|1|For development|
|docker-compose-nuc.yaml|simulation|x86_64|0|For development|
|docker-compose-jetson.yaml|simulation|arm64|1|For development|
|docker-compose-production.yaml|real|x86_64 + NVIDIA GPU|1|
|docker-compose-nuc-production.yaml|real|x86_64|0|
|docker-compose-jetson-prod.yaml|real|arm64|1|Including built workspace|
|docker-compose-mapping.yaml|real|x86_64|0|

### Packages

|Package|Description|
|---|---|
|[../cabot](cabot)|This package includes cabot basic functions|
|[../cabot_bt](cabot_bt)|cabot behavior trees (BT), BT plugins, and some utilities|
|[../cabot_debug](cabot_debug)|debug utilities for logging output of command to check CPU/GPU status|
|[../cabot_description](cabot_description)|robot URDF description for ROS1|
|[../cabot_description2](cabot_description2)|robot URDF description for ROS2|
|[../cabot_gazebo](cabot_gazebo)|robot launch files for gazebo environment, counter part of cabot package|
|[../cabot_mf_localization](cabot_mf_localization)|launch file and script for launching multi floor localization using RF signals (WiFi/BLE) and cartographer|
|[../cabot_msgs](cabot_msgs)|cabot message definition|
|[../cabot_navigation](cabot_navigation)|this is for old version. navigation package is moved to cabot_navigation2 (ROS2)|
|[../cabot_navigation2](cabot_navigation2)|cabot core navigation logic using Nav2, which works with cabot_ui_manager (ROS1)|
|[../cabot_people](cabot_people)|launch file and script for launching people tracking nodes|
|[../cabot_ros_backpack](cabot_ros_backpack)|message difinitions for backpack module (optional)|
|[../cabot_sites](cabot_sites)|place cabot site packages for ROS1 under this directory|
|[../cabot_sites2](cabot_sites2)|place cabot site packages for ROS2 under this directory|
|[../cabot_ui](cabot_ui)|user interface related code and i18n files|
|[../cabot_util](cabot_util)|utility scripts for ROS2|
|[../cabot_wireless_simulator](cabot_wireless_simulator)|wireless (WiFi / BLE beacons) simulator for gazebo|
|[../mf_localization](mf_localization)|multi floor localization function.|
|[../mf_localization_gazebo](mf_localization_gazebo)|gazebo utility to test multi floor localization|
|[../mf_localization_mapping](mf_localization_mapping)|mapping function for multi floor localization.|
|[../mf_localization_msgs](mf_localization_msgs)|message difinitions for multi floor localization|
|[../mf_localization_rviz](mf_localization_rviz)|rviz plugins for multi floor localization control (floor up/down, restart localization)|
|[../motor_controller](motor_controller)|motor driver and adapter|
|[../nav2_action_bridge](nav2_action_bridge)|action bridge from ROS1 to ROS2|
|[../predict_people_py](predict_people_py)|Kalman filter to predict people velocity|
|[../queue_people_py](queue_people_py)|publish queue message to control robot in queue|
|[../queue_utils_py](queue_utils_py)|utilities for queue|
|[../track_people_cpp](track_people_cpp)|detect and track people (cpp implementation) for improved performance|
|[../track_people_py](track_people_py)|detect and track people|
|[../wireless_scanner_ros](wireless_scanner_ros)|WiFi/BLE scanner for ROS|

## Other directory

|Package|Description|
|---|---|
|[doc](doc)|documentation|
|[../docker](docker)|docker context and home directory to be mounted|
|[../host_ws](host_ws)|workspace for ROS running on host machine (mainly for debug)|
|[../script](script)|launch scripts for docker containers, utilities to plot from bag file|
|[../tools](tools)|install/setup scripts|
