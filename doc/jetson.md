# CaBot with Jetson configuration

CaBot is originally designed for a PC with a NVIDIA GPU (small game PC), but the GPU consumes a lot of power.
So, this configuration uses NVIDIA Jetson, trying to reduce power usage.

## Hardware
- NUC (Ruby R8) + Jetson Xavier AGX
- NUC (Ruby R8) + Jetson Mate (1~4 Jetson Xavier NX)

## Network
- all Jetson should be configured in a same local network

## Jetson Mate stroage
- Jetson Mate does not provide external storage, so the system can only use 16GB RAM on Jetson Xavier NX
  - To save disk usage, need to carefully install minimum requirement. To manage installation, Jetson setup scripts is privided.

## Jestson setup scripts
TBD

## Build Docker images
- Install docker arm emulator if you build l4t(jetson) image on x86_64 machine
  ```
  ./install-arm-emulator.sh
  ```
- build image
  ```
  ./prebuild-docker.sh l4t && ./build-docker.sh l4t 
  ```
- build production images for jetson to eliminate all build steps including building workspace 
  ```
  ./build-production-image.sh
  ```

## Launch people service on jetson

- see `jetson-launch.sh -h` for details

