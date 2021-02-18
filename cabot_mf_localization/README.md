
## Work with cabot_ros

### Required repositories
- [cabot_ros](https://github.com/CMU-cabot/cabot_ros)
- cabot site repositories
  - e.g. [cabot_sites](https://github.com/CMU-cabot/cabot_sites)

### Multi-floor localization on Gazebo simulator
In first terminal, run cabot.sh with simulation (-s), minimum (-i) and no-rviz (-O) options. -Z option is required to specify z-position of the robot in gazebo.
```
$ cd ~/catkin_ws/src/cabot_ros/script
$ ./cabot.sh -r YOUR_ROBOT -d -s -i -z -O -T YOUR_CABOT_SITE -Z 10.0
```

In second terminal, run multi-floor localization with simulation (-s) option.
```
$ roscd cabot_mf_localization/script
$ ./cabot_mf_localization.sh -d -s -T YOUR_CABOT_SITE
```

### Multi-floor localization on a physical robot
In first terminal, run cabot.sh with minimum (-i) and no-rviz (-O) options.
```
$ cd ~/catkin_ws/src/cabot_ros/script
$ ./cabot.sh -r YOUR_ROBOT -d -i -z -O -T YOUR_CABOT_SITE
```

In second terminal, run ble_scanner.js to start scanning BLE RSS values.
```
$ roscd /wireless_scanner_ros/script/scanner
$ sudo ./ble_scanner.js
```

In third terminal, run multi-floor localization.
```
$ roscd cabot_mf_localization/script
$ ./cabot_mf_localization.sh -d -T YOUR_CABOT_SITE
```
