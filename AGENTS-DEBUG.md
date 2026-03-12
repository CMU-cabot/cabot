# AGENTS-DEBUG.md

## Scope

- This file supplements `AGENTS.md` for debug and Playwright-related work.

## Debug Image

- The Playwright-enabled debug image is built by:
  - `./bake-docker.sh -i debug`
- The resulting local image is `cmucal/cabot-debug:latest` unless a different tag is specified by the build flow.

## Playwright Workflow

- `box_download_logs.mjs` is intended to run in the `debug` service, not on the host directly.
- Typical invocation:
  - `docker compose exec -u developer debug node /home/developer/src/cabot/cabot_debug/script/box_download_logs.mjs <github-issue-url>`
- The helper script for downloaded archives is:
  - `cabot_debug/script/download-helper.sh`
- This workflow is primarily for downloading logs from a specified GitHub Issue and then using those logs for debugging.

## Validation

- For Playwright changes, verify inside the `debug` container rather than assuming host behavior.
- If a browser automation flow depends on persistent login, reuse the profile under `~/Downloads/.box-profile` inside the container.

## Improvement Ideas

- Consider storing the Issue text together with the downloaded logs for later debugging context.
