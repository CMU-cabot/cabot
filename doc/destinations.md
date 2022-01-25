# CaBot Destinations

- CaBot destination is managed by [hulop MapService](https://github.com/hulop/MapService)
- Destinations can be edited usually at `https://<your domain>/map/editor.jsp`


# Navigation Procedure

1. a user requests the robot for navigating to a destination on an UI (i.e, smartphone app, voice command)
1. the system publishes a `std_msgs/String` message like `navigation;destination;<node_id>` to `/cabot/event` topic
  ```
  $ rostopic pub -1 /cabot/event std_msgs/String "data: 'navigation;destination;<node_id>'"
  ```
1. the cabot obtains a route to the destination from the MapService server
1. the cabot devides a route into sub routes depending on its building structure (i.e, manual door, elevator)
1. tha cabot manages the subgoals based on its location and send each subgoal to navigation2


# List of Destinations (an example at CMU site)

- you can get landmark.json with the following commands
  ```
  $ curl 'http://cmu-map.mybluemix.net/map/routesearch' \
    -H 'Content-Type: application/x-www-form-urlencoded; charset=UTF-8' \
    --data-raw 'action=start&cache=false&lat=40.443355355898206&lng=-79.94565317615195&dist=500&user=test&lang=en' \
    --output landmark.json
  ```
- then list all destinations on the 4th floor from the landmark.json file (CMU gazebo world supports only 4th floor)
  ```
  $ jq -r '.landmarks[] | select(.name!="" and .properties.ent1_fl==4) | .name + ", " + .node' landmark.json
  ```

# Implementation detail
- UI related events are managed on `/cabot/event` topic
- `/cabot/event` (std_msgs/String) will be parsed [here](https://github.com/CMU-cabot/cabot/blob/a8a77f48d23fcb83d2bf5b80a0a567f9ccd91bc6/cabot_ui/src/cabot_ui_manager.py#L198) and navigation related events are handled [here](https://github.com/CMU-cabot/cabot/blob/a8a77f48d23fcb83d2bf5b80a0a567f9ccd91bc6/cabot_ui/src/cabot_ui_manager.py#L282). Examples of events are as follows.
  ```
  navigation;destination;<node_id>                  # start navigation to the destination
  navigation;cancel                                 # cancel navigation
  ```
- `<node_id>` can be any id of node in the topology on the MapService map. The all nodes should be connected as one graph. Otherwise, the service may not able to get a proper route.

## User Interface
- Initially, the UI is built up on a voice-feedback menu with buttons on robot's handle, but it is not maintained.
- The current main UI is built on iOS (not publicly available yet)
  - The app will communicate with the robot through a [BLE server](https://github.com/CMU-cabot/cabot/blob/dev/cabot_ui/src/cabot_ble.py). The app will be a peripheral of the robot.