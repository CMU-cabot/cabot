# Local Map Service

CaBot launch a [MapService](https://github.com/hulop/MapService) Web server if server data exists in CABOT_SITE dir.

## Limitations

- This server can be accessed only from localhost
- The script will use [admin's default password](https://github.com/hulop/MapService/blob/master/MapService/SETUP.md#administration)
- The script will create editor role account 'editor' with password 'editor'
- Mongodb's data is stored on container disk. The data is not persistent and cleared every launch.

## Start the server
```bash
./server-launch.sh -d <data_dir>
```

## Export route data
```bash
./tools/server-data.sh -e <file>
```
- You can also export the data with Web editor ([link to local server editor page](http://localhost:9090/map/editor.jsp))
- You need to use admin page to export `attachments.zip` ([link to local server admin page](http://localhost:9090/map/admin.jsp))

## Server data files


```text
- CABOT_SITE/                     # CABOT_SITE is an identical directory name under ./cabot_sites dir
  |
  |- server_data/
  | |
  | |- MapData.geojson             # exported route data (editor.jsp)
  | |- attachments                 # unzip exported attachments.zip file (admin.jsp) into this dir
  | |- server.env                  # server environment variables
  |
  |- config/config.yaml            # set values for localhost
       map_server_host: localhost:9090/map
       protocol: http
```

- link [server environment variables](https://github.com/hulop/MapService/blob/master/MapService/SETUP.md)
