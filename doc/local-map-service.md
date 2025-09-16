# Local Map Service

CaBot launch a [MapService](https://github.com/hulop/MapService) Web server for the specified `CABOT_SITE`.
The MapService server will be accessed from cabot-navigation system and cabot-ios-app.
The MapService server provides topological route and POI data for navigation, and destination and tour data to be displayed in the app.

## Server data files

```
- CABOT_SITE/                     # CABOT_SITE is an identical directory name under ./cabot_sites dir
  |
  |- server_data/
  | |
  | |- MapData.geojson             # exported route data (editor.jsp)
  | |- attachments                 # unzip exported attachments.zip file (admin.jsp) into this dir
  | | |- cabot/tourdata.json       # tour data for cabot-ios-app
  | | |- map
  | | | |- floormaps.json          # floor map data for MapService
  | | | |- <map image files>       # floor map images specified in floormaps.json 
  | |- server.env                  # server environment variables
  |
  |- config/config.yaml            # set values for localhost
        map_server_host: localhost:9090/map
	      protocol: http
```

- link [server environment variables](https://github.com/hulop/MapService/blob/master/MapService/SETUP.md)

## Login

- The script will use [admin's default password](https://github.com/hulop/MapService/blob/master/MapService/SETUP.md#administration)
- The script will create editor role account 'editor' with password 'editor'

## Server Data Persistency

- Mongodb's data is stored on container disk. The data is not persistent and cleared every launch.

## Start the server
```
./server-launch.sh -d -p <CABOT_SITE>                # -d for development (`CABOT_SITE` data should be under `cabot/cabot-navigation/cabot_sites/`)
./server-launch.sh -p <CABOT_SITE>                   # for production; `CABOT_SITE` data should be built and under `cabot/cabot-navigation/cabot_site_pkg`
```

## Edit and export routes/POIs (MapData.geojson)

- [http://localhost:9090/map/editor.jsp](http://localhost:9090/map/editor.jsp) (for route and POI data)

### Edit route based on robot's location

Useful when the robot can localize and you want to create routes based on the robot's live position while manually moving it on site.
(You move the robot physically while connecting to the robot's MapService from a PC and edit on the spot.)

- Launch the robot with `CABOT_POST_LOCATION=true`
  - check "Show Robot Location" in the editor page
  - the robot's location will be shown as a green circle with a line indicating the heading direction

### Edit route based on robot's location history

Useful when the robot can localize and you want to create routes later based on the history of where you manually moved it.
(You don't edit on site; instead you move the robot to record the path and later use the recorded history to build routes.)

- Launch the robot with `CABOT_POST_LOCATION=true`
  - the robot's location history will be recorded and can be used to create routes
  - stop the robot system
- Or use the recorded bag file to post location data
  ```
  ./build-workspace.sh -o
  source host_ws/install/setup.bash
  ros2 run cabot_debug post_locations.py -f <bag file> -c cabot_debug -S 10   # post every 10th location message with client=cabot_debug
  ```
- Access the MapService log view page
  - [http://localhost:9090/map/logview.jsp](http://localhost:9090/map/logview.jsp) (for server log view)
  - Download the location log file
- Load Robot Location in the editor page
  - the robot's location history will be shown as a series of green circles with lines indicating the heading direction


## Edit and export tours (tourdata.json)

- [http://localhost:9090/map/tour-editor.jsp](http://localhost:9090/map/tour-editor.jsp) (for tour data)

