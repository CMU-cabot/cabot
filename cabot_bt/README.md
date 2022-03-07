# cabot_bt package (ROS2)

cabot [behavior trees](https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree) (BT), BT plugins, and some utilities


## behavior trees

behavior tree|description
---|---
navigate_for_elevator.xml |BT for entering elevator
navigate_for_queue.xml |BT for queue navigation
navigate_for_queue_exit.xml |BT for after queue navigation
navigate_w_local_odom.xml |BT for exiting elevator (do not use global map)
navigate_w_replanning_and_recovery.xml |BT for basic navigation

## plugins
### action
|plugin|node name|description|
|------|---------|-----------|
|change_param.cpp|ChangeParam|change parameter of node to value|
|clear_costmap_service.cpp|ClearCostmapAroundRobot|call ClearCostmapAroundRobot service|
|current_pose.cpp|CurrentPose|get current robot pose|
|ignore_people.cpp|IgnorePeople|specify avoiding people (ignore from costmap exclusion)|
|path_to_poses.cpp|PathToPoses|extrace poses from the path|
|publish_topic.cpp|PublishTopic|publish String topic|
|                 |PublishTopicInt8|publish Int8 topic|
|                 |PublishTopicPath|publish Path topic|
|                 |PublishTopicPoseStamped|publish PoseStamped topic|
|wait_float.cpp|WaitFloat|wait for seconds specified by a float value (for less than 1.0 seconds)|

### condition
|plugin|node name|description|
|------|---------|-----------|
|check_path.cpp|CheckPath|check if planned path is not substantially detour from target path|
|need_to_avoid.cpp|NeedToAvoid|check if standing person is on the path|
|people_exist.cpp|PeopleExists|check existence of detected people|
|person_stops.cpp|PersonStops|check if there is standing people|
|can_pass_person.cpp||not used|
|far_enough.cpp||not used|
|someone_not_in_queue.cpp||not used|
|someone_on_path.cpp||not used|

### decorator
|plugin|node name|description|
|------|---------|-----------|
|restore_config.cpp|RestoreConfig|stores all config of the specified node when enter and restore when exit|

## Utilities
- check_eval: print controller evaluation message in rosbag2 file
- check_log: print BT navigator log message in rosbag2 file

