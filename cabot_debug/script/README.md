Playwright utility script

- box_download_logs.mjs - Download Box log files linked from a GitHub issue comment via Box web UI
  - Requirements:
    - Build the debug image (`cabot-navigation/docker/ros2/Dockerfile.debug`): `docker compose --profile dev build debug`
    - Start the container: `docker compose --profile dev up -d debug`
    - If `XAUTHORITY` is not exported on the host, generate a fallback file once: `./tools/setup-display.sh`
  - Run:
    - `docker compose exec -u developer debug node /home/developer/src/cabot/cabot_debug/script/box_download_logs.mjs <github-issue-url>`
    - `docker exec -u developer -it $(docker compose ps -q debug) node /home/developer/src/cabot/cabot_debug/script/box_download_logs.mjs <github-issue-url>`
  - Behavior:
    - Persistent browser profile: `~/Downloads/.box-profile`
    - Download destination: `~/Downloads`
    - Opens the GitHub issue, reads the first comment/body, opens the Box folder link there, and downloads the listed files from the folder view
    - If GitHub or Box shows a login page, pauses for manual login in the browser
    - Starts `cabot_debug/script/download-helper.sh` in parallel and waits until extraction finishes under `~/src/cabot/docker/home/sandbox/<owner>-<issue-number>`
