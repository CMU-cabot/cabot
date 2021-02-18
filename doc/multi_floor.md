# Multi-floor global localization

A ROS package for multi-floor global localization using LIDAR, IMU, and BLE RSS.

## How to create data for multi-floor localization

## Data Collection
### Collect bag files in your target environment
#### Repeat the following steps for each area in the environment
1. Start WiFi and BLE scanner nodes by following the [instructions](https://github.ibm.com/trl-ar/wireless_scanner_ros#usage)
2. Run realtime cartographer with recieving WiFi/BLE data and recording required topics to a bag file.
(Replace BAG_PREFIX to what you like)
```
$ roslaunch mf_localization_mapping realtime_cartographer_2d_VLP16.launch \
    record_wireless:=true save_samples:=true record_required:=true \
    bag_filename:=BAG_PREFIX_`date +%Y-%m-%d-%H-%M-%S`
```
  * When the size of the area is very large and realtime cartographer does not run normally, add ```run_cartographer:=false``` as a command line argument to disable cartographer during data collection.
  * To check if the required topics are published or not during data collection, open another terminal and run
     ```
     $ rosrun mf_localization_mapping topic_checker.py
     ```
     If it prints ```Setup completed``` in the terminal, the required topics are published.

## Date Processing
### Reconstruct floor maps and RSS fingerprint data from the collected bag files
#### Repeat the following steps for each bag file
1. Convert a recorded bag file to the cartographer-friendly format. (Replace BAG_FILENAME to what you like)
```
$ roslaunch mf_localization_mapping convert_rosbag_for_cartographer.launch \
    convert_points:=true bag_filename:=BAG_FILENAME.bag
```
This command produces one file (BAG_FILENAME.bag.carto-converted.bag).

2. Reconstruct a pose graph and RSS fingerprint from the converged bag file.
```
$ roslaunch mf_localization_mapping demo_2d_VLP16.launch \
    save_samples:=true bag_filename:=BAG_FILENAME.bag.carto-converted.bag
```
This command produces two files (BAG_FILENAME.bag.carto-converted.bag.pbstream and BAG_FILENAME.bag.carto-converted.bag.loc.samples.json).

3. Create a 2D-projected map image from the pose graph file
```
$ rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename BAG_FILENAME.bag.carto-converted.bag.pbstream \
    -map_filestem BAG_FILENAME.bag.carto-converted
```
This command produces two files (BAG_FILENAME.bag.carto-converted.pgm and BAG_FILENAME.bag.carto-converted.yaml).

<!--
3. Compose a pointcloud map file from the converted bag file and the pose graph file.
```
$ roslaunch mf_localization_mapping assets_writer_megarover_3d.launch \
    bag_filenames:=BAG_FILENAME.bag.carto-converted.bag \
    pose_graph_filename:=BAG_FILENAME.bag.carto-converted.bag.pbstream
```
4. Project the pointcloud map into 2d images after preprocessing the file by the voxel grid filter and file format conversion (binary to ascii).
```
$ roscd sensor-tools/script/cloud
$ ./project_pcd_2d.py --preprocess -i BAG_FILENAME.bag.carto-converted.bag_points.pcd -o BAG_FILENAME.bag.carto-converted.bag_points_ds0.1_ascii
```
-->

### Align the floor maps in the global coordinate
1. Upload the projected image file to LocationTools.
  * It is necessary to convert a pgm image to png image before uploading it. The original pgm file must be preserved because it is used later.  
  * To extract floor map information (origin_x, origin_y and ppm) from yaml files, use the following tool.
    ```
    $ rosrun mf_localization_mapping extract_floor_map_info_from_yaml.py -i BAG_FILENAME.bag.carto-converted.yaml
    ```
2. Align the projected images to floor plans
3. Obtain the information of latitude, longitude, and rotate for the projected images

### Organize the pose graph and fingerprint files for multi-floor localization
1. Create a ROS package to store data (e.g. cabot_site_name)
2. Create ```maps``` directory in cabot_site_name package
  ```
  $ roscd cabot_site_name
  $ mkdir maps
  ```
3. Copy the pose graph (.pbstream), RSS fingerprint (.loc.samples.json), pgm map (.pgm) and map configuration (.yaml) files to ```maps``` directory
4. Create a yaml config file in maps directory in the following format (e.g. ```cabot_site_name.yaml```)
   - Example
    ```
    anchor:
      latitude: 40.4432971
      longitude: -79.9463286
      rotate: 30
      floor: 0
    map_list:
      - node_id: "carto_0_0"
        frame_id: "map_carto_0_0"
        latitude: 40.4430307
        longitude: -79.945827
        rotate: 60
        floor: 0
        area: 0
        load_state_filename: "~/catkin_ws/src/cabot_site_name/maps/floor0_area0.pbstream"
        samples_filename: "~/catkin_ws/src/cabot_site_name/maps/floor0_area0.loc.samples.json"
        map_filename: "~/catkin_ws/src/cabot_site_name/maps/floor0_area0.yaml"
      - node_id: "carto_0_1"
        ...
    ```

## How to run the multi-floor localization package

### Demo to run a bag file
```
$ roslaunch mf_localization_mapping demo_multi_2d_VLP16_rss_localization.launch beacon_topic:=wireless/beacons map_config_file:=~/catkin_ws/src/cabot_site_name/maps/cabot_site_name.yaml bag_filename:=YOUR_BAG_FILE_FOR_DEMO.bag
```
