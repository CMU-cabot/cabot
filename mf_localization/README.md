# mf_localization package (ROS1)

Provide multi floor localization function.

## configuration_files

configuration files for cartographer

## src/multi_floor_manager.py

multi_floor_manager launches multiple cartographer nodes for each floor or area in map configuration and reroute topics into a selected cartographer to control multi floor/area localization.
To select a specific cartographer (floor/area) use radio frequency signals or other sensors.

### publish
- **current_floor**: current floor
- **current_floor_raw**: current floor estimation before rounding
- **current_frame**: current floor/area's frame id
- **current_map_files**: current floor/area's map file name
- **resetpose**:
- **global_position**:
- **scan_matched_points**:

### subscribe
- **imu**: input for cartographer
- **points2**: input for cartographer
- **beacons**: to estimate floor/area 
- **initialpose**: to set initial pose
- **odom**: input for cartographer
- **pressure**: to check if robot is move vertically

### TF
- **map_global**: the global TF origin
  - **-> map_carto_XX**: each cartographer map origin
- **map_carto_XX -> map**: for ROS2, map refers current_frame

## src/multi_floor_map_server.py

multi_floor_map_server launches multiple map server nodes for each floor or area in map configuration and reroute map topic from the map server of the current_frame

### publish
- **map**: map corresponds to the current_frame

### subscribe
- **current_frame**: 
