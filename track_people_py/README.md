# track_people_py package (ROS1)

detect and track people

## scripts/detect_darknet_people.py, detect_darknet_opencv_people.py

detect people using darknet, and opencv implementation of darknet in RGB image and estimate position from depth image

### yolov4 model

`../tools/setup-model.sh` to download yolov4.weights file

### publish
- **/track_people_py/detected_boxes**: detected people location without track id

### subscribe
topic name can be changed by parameter

- **/camera/color/camera_info**: to get camera size
- **/camera/color/image_raw**: RGB image
- **/camera/aligned_depth_to_color/image_raw**: Depth image

### service
- **enable_detect_people**: to control whether detecting people


## scripts/track_sort_3d_people.py

find corresponding person in detected people to assign track id for prediction

### publish
- **/track_people_py/tracked_boxes**: tracked people location with track id

### subscribe
- **/track_people_py/detected_boxes**: detected people

