# cabot_navigation2 package (ROS2)

cabot core navigation logic using Nav2, which works with cabot_ui_manager (ROS1)

## navigation flow

1. ROS1 cabot_ui_manager (see cabot_ui) receives a navigation destination event
2. cabot_ui_manager query [MapService server](https://github.com/hulop/MapService) a route
3. cabot_ui_manager devide the route into sub routes based on building structure (door, elevator, ...)
4. cabot_ui_manager send a sub goal to ROS2 bt_navigator as well as publish original route to `/path` topic
5. execute the specified behavior tree in the goal (see cabot_bt)

## launch

- bringup_launch.py: navigation2 launcher

## params

- nav2_params.yaml  : parameter for navigation2
- nav2_params2.yaml : parameter for navigation2 without global map (only local map), used for elevator exit

## plugins

- cabot_simple_goal_checker.cpp
  - a [GoalChecker](https://github.com/ros-planning/navigation2/blob/main/nav2_core/include/nav2_core/goal_checker.hpp) plugin
  - check if distance to the goal is under threthold or the goal is behind the robot
- navcog_path_layer.cpp
  - costmap layer to indicate navcog path
- navcog_path_planner.cpp
  - adjust path passed to /path topic
- navcog_path_util.cpp
  - utility for navcog path
- people_obstacle_layer.cpp
  - costmap layer to deal with (include/exclude) /people topic
- pose_align.cpp
  - a dwb [TrajectoryCritic](https://github.com/ros-planning/navigation2/blob/30b405c58e6d53ba8c96381416bc4679d35a1483/nav2_dwb_controller/dwb_core/include/dwb_core/trajectory_critic.hpp) plugin
- util.cpp
  - geometry utilities

## src

- cabot_scan.cpp - not used
