# Requirements

- Ubuntu and ROS
  - Ubuntu16.04 - ROS kinetic
  - Ubuntu18.04 - ROS melodic
- ZED library
- CUDA
- cartographer, cartographer_ros (optional)

# Setup

- import thirdparty repos by using vcstool
```
pip3 install vcstool # if you don't have vcs
cd thirdparty
vcs import < thirdparty.repos
```
- run all script in tools
- run prebuild.sh
- run build.sh

# IBM Watson API key (optional)

If you want to let the robot speak, [IBM Watson TTS API key](https://cloud.ibm.com/apidocs/text-to-speech) is required.
Copy API key to `iam_apikey` entry in `cabot_sites/cabot_site_cmu/config/config.yaml`

# Example usage (on Docker)

```
cd docker
./prebuild-docker.sh
./docker-build.sh
docker-compose up
```

prepare .env file

- set your host computer's IP

# build own cabot site

TBD

# Use Cartographer

Check installation document
https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation