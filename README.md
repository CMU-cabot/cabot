![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

# CaBot

CaBot (Carry on Robot) is an AI suitcase to help people with visually impairments travel independently. Can you imagine to walk around at airports without vision? It’s huge open space and there are lots of things and people, so that it is really dangerous for them to walk around at airports. [see project detail](https://www.cs.cmu.edu/~NavCog/cabot.html)

## CaBot v2

CaBot v2 uses ROS1, ROS2, and ros1_bridge to use [navigation2](https://github.com/ros-planning/navigation2) package for ROS2 and existing packges for ROS1. Also, it uses Docker container to maintain development/production systems. 

### Tested Environment

- Ubuntu 16.04 / 20.04
- Docker v19
- docker-compose v1.25
- Docker containers
  - `ros1`: Ubuntu18.04, ROS1 melodic
  - `ros2`: Ubuntu20.04, ROS2 foxy
  - `bridge`: Ubuntu20.04, ROS1 noetic, ROS2 foxy

## Setup

- import thirdparty repos by using vcstool
```
pip3 install vcstool # if you don't have vcs
cd thirdparty
vcs import < thirdparty.repos
```
- run all script in tools based on your requirements
```
tools/setup-display.sh        # for display connections from docker containers
tools/install-docker.sh       # if you need docker
tools/setup-usb.sh            # if you run physical robot
```
- build docker containers
```
cd docker
./prebuild-docker.sh
./docker-build.sh
```
- prepare .env file
  - set your host computer's IP
- run containers. This will show up Rviz. 
```
docker-compose up
```

### Navigate CaBot on Gazebo simulation

- `Navigation 2 Goal` tool does not work properly. 
- You need to use CaBot menu instead, find `xterm` terminal displaying `type 'j', 'k', or 'l' for 'up', 'center', 'down' buttons`
- Type `k`, `k`, `k` to start navigation.

## Customization

See [customization](doc/costomization.md) for more details.

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