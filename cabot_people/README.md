# cabot_people package (ROS1)

launch file and script for launching people tracking nodes

```
Usage
    run this script after running cabot.sh in another terminal
ex)
./cabot_people.sh -O -T <site_package>

-h                       show this help
-d                       debug
-V                       show rviz
-m <map file>            specify a map file
-n <anchor file>         specify a anchor file, use map file if not specified
-w <world file>          specify a world file
-s                       specify its on simulation (gazebo)
-r                       launch realsense camera
-q                       use queue detector
-T <site package>        packge name for the robot site (only for queue)
-p                       publish simulation people instead of detected people from camera
-K                       use people tracker
-D                       use people detector
-C                       check required before launch
-W                       wait roscore
-t <roll>                publish map camera_link tf
-v [0-3]                 use specified opencv dnn implementation
   0: python-darknet, 1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet
-N <name space>          namespace for tracking
-f <camera_link_frame>   specify camera link frame
-F <fps>                 specify camera fps
-S <camera serial>       specify serial number of realsense camera
-R 1280/848/640          specify camera resolution
```
