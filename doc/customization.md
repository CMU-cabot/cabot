# Customization (build your own map)

## Build own cabot site for your environment

- ROS1
  - Required components
    - [config files](map-config-format)
    - localization map/data for Cartographer
    - static map image for Rviz
    - MapService server data (local/remote)
  - Optional components
    - world files for gazebo simulation
    - localization map/data and static map images made for gazebo worlds
    - menu configuration and i18n strings
- ROS2
  - Required components
    - config files
    - static map image for ROS2 Navigation2
  - We usually make a separate branch which has modified package.xml and CMakeLists.txt for ROS2

### Example

- See [example cabot site for CMU campus](https://github.com/CMU-cabot/cabot_sites_cmu/tree/main/cabot_site_cmu_3d)

### Mapping with Cartographer
- You need to walk around your environment with your robot or a device equipped suitcase to scan and build the map the place
  - walk slowly (less than 1.0m/s) and cover entire space
  - it would be better to have a round trip for all possible topology (corridors, rooms, spaces separated by large objects, and etc)
  - example of a device-equipped suitcase (using a [camera mount with a clamp](https://www.smallrig.com/smallrig-crab-shaped-clamp-magic-arm-with-ball-head-3724.html), IMU device would be mounted on the LiDAR)
    - <img alt="a suitcase equipped with a lidar" src="suitcase_for_mapping.jpg" width="240"/>
- Required data, devices, and software
  - **Point clouds**: Velodyne VLP16
  - **IMU**: Xsens, or BNO055 managed by Arduino or ESP32)
    - see [xsens_driver](https://wiki.ros.org/xsens_driver) for compatible devices
    - [Code for Arduino + BNO055](https://github.com/CMU-cabot/cabot-arduino)
      - tested with [Arduino Mega](https://store.arduino.cc/products/arduino-mega-2560-rev3)
    - [Code for ESP32 (WiFi) + BNO055](https://github.com/CMU-cabot/WiFiScan/tree/wifi-imu-class)
      - tested with [SparkFun Thing Plus](https://www.sparkfun.com/products/15663) + [BNO055](https://www.adafruit.com/product/4646) connected by a [Qwiic cable](https://www.adafruit.com/product/4399)
  - **WiFi signals**: ESP32 compatible device with WiFi antenna
    - [Code for ESP32 (WiFi)](https://github.com/CMU-cabot/WiFiScan)
      - tested with [ESP32 devkitc-v4](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html)
  - **Bluetooth signals**: PC with Bluetooth
  - **CPU**: PC for Ubuntu20.04
- Procedure
  - [build docker images](../README.md#build-docker-images)
  - start scannning and walk
    ```
    $ ./mapping-launch.sh -o TEST1              # use ESP32 for IMU with prefix TEST1
    $ ./mapping-launch.sh -o TEST2 -x           # use XSENS for IMU with prefix TEST2
    $ ./mapping-launch.sh -o TEST3 -a           # use Arduino for IMU with prefix TEST3
    ```
    - these commands record topics into a bag file for post processing
    - the bag file started with the prefix you specified can be found under docker/home/.ros

  - run post processes the bag file (would be better to use PC with at least 6 core and 16GB)
    ```
    $ ./mapping-launch.sh -p <bag file>
    $ ./mapping-launch.sh -p <bag file> -w     # if the bag file is more than a few minitues, this option would be better
    $ ./mapping-launch.sh -p <bag file> -w -n  # the script will not skip previously completed tasks
    ```
    - post processes consist of 1) converting the point clouds to a laser scan 2) running cartographer to SLAM 3) making a pgm image file from cartographer submaps
    - you can find the result under docker/home/post_process (the specified bag file will be copied here)

- Issues with mapping a large environment?
  - Please consult at [Issues](/issues), you may need to configure cartographer params to get a better result

- Align the map to global coordinate (TBD)

### MapService server

- [MapService](https://github.com/hulop/MapService) repository on github HULOP project
- about [local MapService server](local-map-service.md)

## IBM Watson API key (optional)

If you want to let the robot speak, [IBM Watson TTS API key](https://cloud.ibm.com/apidocs/text-to-speech) is required.
Copy API key to `iam_apikey` entry in `cabot_sites/cabot_site_cmu/config/config.yaml`
