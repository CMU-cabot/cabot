### WiFi/BLE scanner for ROS

#### Dependencies
* rosbridge_server
 * python-tornado<=4.5.3
* Node.js
* npm

#### Installation
1. Install required packages
```
$ sudo apt-get install ros-${ROS_DISTRO}-rosbridge-suite nodejs-legacy npm
$ cd ~/catkin_ws/src/wireless_scanner_ros
```
2. Install python packages
```
$ pip install --user -r requirements.txt # for rosbridge
$ sudo apt-get install python3-requests python3-gi python3-dbus # for dbus_ibeacon_scanner.py
```

3. Install node.js packages
```
$ cd ~/catkin_ws/src/wireless_scanner_ros
$ npm install
```

#### Usage
1. Launch a wifi/ble topic receiver
```
$ roslaunch wireless_scanner_ros wifi_ble_receiver.launch
```

2. Run wifi/ble scanner scripts

  2.1 Move to the scanner directory
  ```
  $ cd catkin_ws/src/wireless_scanner_ros/script/scanner
  ```
  2.2 Open a new terminal window, run a WiFi scanner
  ```
  $ sudo ./wifi_scanner.js -i <network_interface>
  ```
  or
  ```
  $ sudo -s
  $ rosrun wireless_scanner_ros iw_wifi_scanner_node.py _interface:=<network_interface>
  ```

  2.3 Open a new terminal window, run a BLE beacon scanner
  ```
  $ sudo ./ble_scanner.js
  ```
  or
  ```
  $ rosrun wireless_scanner_ros dbus_ibeacon_scanner.py
  ```

3. Check if wifi and ble published to /wireless/beacons and /wireless/wifi topic.
  ```
  $ rostopic echo /wireless/beacons
  ```
  ```
  $ rostopic echo /wireless/wifi
  ```

4. After collecting wifi/ble RSS data, stop each ROS node and node.js script by pressing Ctrl+C.

#### Published topics
  * /wireless/wifi
  * /wireless/beacons
  * /wireless/wifi_scan_str
  * /wireless/beacon_scan_str
