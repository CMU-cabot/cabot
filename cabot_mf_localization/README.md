# cabot_mf_localization package (ROS1)

launch file and script for launching multi floor localization using RF signals (WiFi/BLE) and cartographer

```
Usage
    run this script after running cabot.sh in another terminal
ex)
./cabot_mf_localization.sh -O -T <site_package>

-h                       show this help
-d                       debug
-m <map file>            specify a map file
-n <anchor file>         specify a anchor file, use map file if not specified
-w <world file>          specify a world file
-s                       specify its on simulation (gazebo)
-O                       performance (no rviz)
-T <site package>        packge name for the robot site
-N                       start ROS1 navigation
-M                       start multi-floor map server
-r <robot>               specify a robot for navigation
-f                       use robot footprint without human for navigation
-R <rate:float>          set publish_current_rate
-X                       do not start localization
-C                       run cartographer mapping
-p                       use pressure topic for height change detection
```