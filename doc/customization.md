# Customization

## Build own cabot driver for your robot

- This repository uses `cabot-drivers` and `cabot-description` repositories for managing hardware components and describing the robot
    - these repositories will be imported by `./setup-dependency.sh` script which uses `vcs` command to import repos described in `dependency.repos` files
    - `cabot-navigation` repo also has dependency to `cabot-description` which is used for gazebo simulation
- You can modify those repositories and this `cabot` repos to build your own cabot-navigation compatible robot
    - simplest way is to fork those repositories to your own space, and modify `dependency.repos` to points to your repo
    - make sure all dependency.repos points to your forked repos
    - you may use `./setup-dependency.sh -c` to remove cloned repo first then run `./setup-dependency.sh` to clone correct one
```
- cabot
  - dependency.repos               <- modify
  - cabot-common
  - cabot-description
  - cabot-drivers
    - dependency.repos             <- modify
    - cabot-common
    - cabot-description
  - cabot-navigation
    - dependency.repos             <- modify
    - cabot-common
    - cabot-description
  - cabot-people
    - cabot-common
```

### requirements

- define own `CABOT_MODEL` for example `cabotX-1`
    - `CABOT_MAJOR` will be first 6 letters of your `CABOT_MODEL` (i.e., `cabot-X`)
- `cabot-descrption/cabot_description/robots/{CABOT_MODEL}.urdf.xacro.xml` (i.e., `cabotX-1.urdf.xacro.xml`)
    - this file will be used in `cabot-drivers` (pyshical robot) and `cabot-navigation` (gazebo simulation)
    - please refer to existing URDF descriptions and check [README of cabot-description](https://github.com/CMU-cabot/cabot-description)
- cabot-drivers will launch `cabot_base` `{CABOT_MAJOR}.launch.py` (i.e., `cabotX.launch.py)
    - you can modify launch script at `cabot-drivers/script/launch_driver.sh`
    - refers to the [README of cabot-drivers](https://github.com/CMU-cabot/cabot-drivers) to see required servies/topics

## Build own cabot site (map) for your environment
- cabot site package
  - Directory structure
    ```
    cabot_site_<name>/
    ├ cabot_site_<name>/
    ├ config/
    │ ├ config.yaml
    │ └ config.sh
    ├ maps/
    ├ server_data/
    └ worlds/
    ```
  - Required components
    - [config files](map-config-format)
    - localization map/data for Cartographer
    - static map image for Navigation2
    - MapService server data (local/remote)
  - Optional components
    - test scripts
    - world files for gazebo simulation
    - localization map/data and static map images made for gazebo worlds

### Example
- See [example cabot site for CMU campus](https://github.com/CMU-cabot/cabot_sites_cmu/tree/dev-ros2/cabot_site_cmu_3d)
- See [example cabot site for test](https://github.com/CMU-cabot/cabot_sites_test/tree/main/cabot_site_test_room)


### Deployment

- place cabot site package under `cabot_sites` directory
- run `./build-docker.sh -w`
- set `CABOT_SITE` to your cabot site package name

## Mapping with Cartographer
- You need to walk around your environment with your robot or a device equipped suitcase to scan and build the map the place
  - walk slowly (less than 1.0m/s) and cover entire space
  - it would be better to have a round trip for all possible topology (corridors, rooms, spaces separated by large objects, and etc)
  - example of a device-equipped suitcase (using a [camera mount with a clamp](https://www.smallrig.com/smallrig-crab-shaped-clamp-magic-arm-with-ball-head-3724.html), IMU device would be mounted on the LiDAR)
    - <img alt="a suitcase equipped with a lidar" src="suitcase_for_mapping.jpg" width="240"/>
- Required data, devices, and software
  - **Point clouds**: Velodyne VLP16, Hesai XT16, or similar LiDAR
  - **IMU**: Xsens, or BNO055 managed by Arduino or ESP32)
    - see [xsens_driver](https://wiki.ros.org/xsens_driver) for compatible devices
    - [Code for Arduino + BNO055](https://github.com/CMU-cabot/cabot-arduino)
      - tested with [Arduino Mega](https://store.arduino.cc/products/arduino-mega-2560-rev3)
    - [Code for ESP32 (WiFi) + BNO055](https://github.com/CMU-cabot/cabot-arduino-ace)
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
    $ ./mapping-launch.sh -o TEST1 -e           # use ESP32 for IMU with prefix TEST1
    $ ./mapping-launch.sh -o TEST2 -x           # use XSENS for IMU with prefix TEST2
    $ ./mapping-launch.sh -o TEST3 -a           # use Arduino for IMU with prefix TEST3
    ```
    - these commands record topics into a bag file for post processing
    - the bag file started with the prefix you specified can be found under `docker/home/recordings`

  - run post processes the bag file (would be better to use PC with at least 6 core and 16GB)
    ```
    $ ./mapping-launch.sh -p <bag file>
    $ ./mapping-launch.sh -p <bag file> -w     # if the bag file is more than a few minitues, this option would be better
    $ ./mapping-launch.sh -p <bag file> -w -n  # the script will not skip previously completed tasks
    ```
    - post processes consist of 1) converting packets topics to pointcloud topics 2) running cartographer for SLAM 3) making a pgm image file from cartographer submaps
    - you can find the result under `docker/home/post_process` (the specified bag file will be copied here)

- Issues with mapping a large environment?
  - run cartographer with reduced rate (like `-r 0.5`), if your computer has smaller number of cores
  - please consult at [Issues](/issues), you may need to configure cartographer params to get a better result

- Align the map to global coordinate
  - launch location tools server
    ```
    $ ./server-launch.sh -l
    $ xdg-open http://localhost:9091/tools
    ```
  - login with [default password](https://github.com/hulop/MapService/blob/master/MapService/SETUP.md#administration)
  - create a new DB for your new site and go to "CaBot Mapping" page for the DB
  - click "Import Mapping Data"
    - copy plain files (do not include bags directories) under `docker/home/post_process` directory to a new directory
    - select the directory to upload
  - edit the anchor of the image (world coordinate)
    - refresh the page make sure the data is the latest
    - click "map" button of a floorplan
    - you can type latitude and longitude if you know the approximate coordinate
    - otherwise, zoom out the map and find your place
    - input rotate value to align the image to the building shape on the map (if available)
    - if you want to use another map image to align a map image, click "map" of the another image first, and then click "map" of the map image you want to align. You can edit the anchor of the image which you "map" last.
    - click "save"
  - export data
    - click "Export Maps (zip)" button to get `maps.zip`
    - go to "Manage Floor Plan" view and click "Export for MapServer" to get `floormaps.zip`
- Setup localization data for your cabot_site
    - edit the following configuration files
      ```
      cabot_site_somewhere
      ├── CMakeLists.txt
      ├── config
      │   ├── config.sh
      │   └── config.yaml
      ├── maps (copy exported `maps.zip`)
      ├── package.xml
      └── server_data
         ├── attachments
         │   └── map (copy exported `floormaps.zip`)
         ├── MapData.geojson
         └── server.env
      ```

- Setup server data for your cabot_site
  - [example](https://github.com/CMU-cabot/cabot_sites_cmu/tree/main/cabot_site_cmu_3d/server_data)
  ```
  MapData.geojson     # routes and POIS, so you may not have one at initially make the data
  server.env          # server environment (you may want to copy from the exemple and change the initial location)
  attachments/map     # defalte the floorplans.zip you generated above
    floormaps.json
    <image files>
  ```
- Edit routes and POIs (TBD)
  - login with editor/editor account (for local setup)
  ```
  $ xdg-open http://localhost:9090/map/editor.jsp
  ```
  - edit routes and POIS
  - export MapData.geojson file and copy to the server_data folder

### MapService server

- [MapService](https://github.com/hulop/MapService) repository on github HULOP project
- about [local MapService server](local-map-service.md)

## IBM Watson API key (optional)

If you want to let the robot speak, [IBM Watson TTS API key](https://cloud.ibm.com/apidocs/text-to-speech) is required.
Copy API key to `iam_apikey` entry in `cabot_sites/cabot_site_cmu/config/config.yaml`
