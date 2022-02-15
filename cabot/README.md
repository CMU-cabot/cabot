# cabot package (ROS1)

This package includes cabot basic functions.

## launch

includes physical robot's launch files. Some of the files are not maintained.

## CaBot basic topics

- /cmd_vel: raw command velocity published from controller
- /odom: odom output for controller
- /cabot/event: UI related events such as button clicks and navigation controls
  - subscriber: [cabot_ui](../cabot_ui)

### topics flow

```
                                                      /cabot/cmd_vel_limited
+================+ /cmd_vel    +===================+              +===============+
|                |============>| * SpeedControl    |=============>| *             |
| Controller     |             +===================+              | OdomAdapter   |
|                |<===============================================|               |
+================+ /odom                                          +===============+
                                                                     A          |
                                                                     |          | /cabot/cmd_vel
                  /cabot/imu/data                                    |          |
+================+             +===================+  /cabot/odometry/filtered  |    
|*Cabot Sensor   |============>| *                 |=================+          |     
|================|             | RobotLocalization |                            |
                               |                   |<==================+        |
                               +===================+                   |        |
                                                       /cabot/odom_raw |        |
                                                                       |        | 
                                                  /cabot/motorStatus   |        v 
+================+             +==================+               +===============+
|                |============>|                  |==============>| *             |
| Motor          |   Serial    | MotorControl     |               | MotorAdapter  |
|                |<============|                  |<==============|               |
+================+             +==================+               +===============+
                                                  /cabot/motorTarget
```


## Safety controller

xxx_speed topics limit the maximum speed of input command velocity

### publish
- /cabot/cmd_vel_limited       - output command velocity

### subscribe
- /cmd_vel                     - input command velocity
- /cabot/lidar_speed           - control by lidar sensor, proximity of obstacle and blind spots
- /cabot/map_speed             - control by map speed poi
- /cabot/people_speed          - control by surrounding people distances
- /cabot/queue_speed           - control by queue manager
- /cabot/tf_speed              - control by existence of map -> base_footprint
- /cabot/touch_speed_switched  - control by touch sensor
- /cabot/user_speed            - control by user setting

## cabot_serial.py
A wrapper of SerialClient of rosserial_python to handle errors and expand packed data
  - [cabot-arduino](https://github.com/CMU-cabot/cabot-arduino) is for microcontroller code using rosserial

### publish

topic|remap|hz|description
---|---|---|---
/cabot/imu|/cabot/imu_raw|100|packed IMU data
/cabot/touch|-|50|1 if the touch sensor is touched otherwise 0
/cabot/touch_raw|-|50|touch sensor raw value (0-255)
/cabot/touch_speed|/cabot/touch_speed_raw|50|2.0 if the touch sensor is touched otherwise 0.0
/cabot/pushed_1|-|50|1 if the front button is pushed otherwise 0
/cabot/pushed_2|-|50|1 if the back button is pushed otherwise 0
/cabot/pushed_3|-|50|1 if the left button is pushed otherwise 0
/cabot/pushed_4|-|50|1 if the right button is pushed otherwise 0
/cabot/pressure|-|2|air pressure (used for localization)
/cabot/temperature|-|2|air temperature (not used)

### subscribe

topic|value|description
---|---|---
/cabot/vibrator1|0-255| front vibrator intensity
/cabot/vibrator2|0-255| (not used)
/cabot/vibrator3|0-255| left vibrator intensity
/cabot/vibrator4|0-255| right vibrator intensity

### topic conversion

topic|remap|description
---|---|---
/cabot/imu|/cabot/imu/data|expanded IMU data
/cabot/touch_speed_switched|-|touch speed output based on the touch active mode

### touch active mode service

- this service switch touch active mode
- /cabot/set_touch_speed_active_mode : SetBool (default = True)
  ```
  # touch speed active mode
  # True:  Touch - go,    Not Touch - no go
  # False: Touch - no go, Not Touch - go
  ```

## cabot_handle_v2_node.py
manages buttons input and vibrator output

### publish

- /cabot/event
  - convert pushed_[1-4] topics into ButtonEvent and ClickEvent

### subscribe

- /cabot/notification
  - convert notification ID into vibration patterns

