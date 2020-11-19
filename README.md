# Requirements

- Ubuntu and ROS
  - Ubuntu16.04 - ROS kinetic
  - Ubuntu18.04 - ROS melodic
- ZED library
- CUDA
- cartographer, cartographer_ros (optional)

# Example usage (on Docker)

```
./build-docker.sh
docker-compose run cabot
./script/cabot.sh -s
```

# Use Cartographer

Check installation document
https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation