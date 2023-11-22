# Route explore annotation rviz plugins
This plugins can annotate intersections indoor space

## Usage

Copy `route_explore_rviz` and `route_explore_msgs` or make symbolic links to them into your catkin workspace src folder. Then build and launch the following command.


- roslaunch route_explore_rviz annotation.launch \
  annotation_file:=<annotation_file.yaml> \
  map_file:=`realpath path_to_map_file`


## Add annotation
- move the location where you want to add an annotation to the center of the view
- click `Add annotation` button
  - `sphere` marker indicates the center of the intersection. You can move by selecting the marker and dragging with the arrows shown.
  - You can increase/decrease number of `ways1 which connected to the intersection by selecting the context menu
  - You can move each `way` by selecting it
- You can delete an annotation by selecting in the list and hit `Delete annotation`
- If you hit `Save annotation`, it will be saved as <annotation_file.yaml> as you specified for launching.