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
docker-compose files mount home directory (`docker/home`) and required CaBot packages into the corresponding workspace to keep built binaries on the host file systems.

`launch.sh` will select the proper docker-compose file to launch.

|File|Real/Simulation|Processor|Realsense|Description|
|---|---|---|---|---|
|docker-compose-common|-|x86_64|0-3|Common declaration|
|docker-compose|simulation|x86_64 + NVIDIA GPU|1|For development|
|docker-compose-nuc|simulation|x86_64|0|For development|
|docker-compose-rs3|simulation|x86_64 + NVIDIA GPU|3|For development|
|docker-compose-jetson|simulation|aarch64|1|For development|
|docker-compose-production|real|x86_64 + NVIDIA GPU|1|
|docker-compose-nuc-production|real|x86_64|0|
|docker-compose-rs3-production|real|x86_64 + NVIDIA GPU|3|3 Realsense|
|docker-compose-jetson-prod|real|aarch64|1|Including built workspace|
|docker-compose-mapping|real|x86_64|0|
|docker-compose-server|both|x86_64|-|local map service server|

## Other directories

|Package|Description|
|---|---|
|[cabot_debug](../cabot_debug)|debug utilities for logging output of command to check CPU/GPU status|
|[cabot_sites](../cabot_sites)|place cabot site packages|
|[doc](../doc)|documentation|
|[docker](../docker)|docker context and home directory to be mounted|
|[host_ws](../host_ws)|workspace for ROS running on host machine (mainly for debug)|
|[script](../script)|launch scripts for docker containers, utilities to plot from bag file|
|[tools](../tools)|install/setup scripts|
