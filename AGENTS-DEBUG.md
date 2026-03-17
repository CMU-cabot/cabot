# AGENTS-DEBUG.md

## Scope

- This file supplements `AGENTS.md` for debug and Playwright-related work.

## Debug Image

- The Playwright-enabled debug image is built by:
  - `./bake-docker.sh -i debug`
- The resulting local image is `cmucal/cabot-debug:latest` unless a different tag is specified by the build flow.

## Playwright Workflow

- `box_download_logs.mjs` is intended to run in the `debug` service, not on the host directly.
- Typical invocation from the repository root inside the `debug` container:
  - `docker compose exec -u developer debug bash -lc 'node cabot_debug/script/box_download_logs.mjs <github-issue-url>'`
- The helper script for downloaded archives is:
  - `cabot_debug/script/download-helper.sh`
- This workflow is primarily for downloading logs from a specified GitHub Issue and then using those logs for debugging.

## Validation

- For Playwright changes, verify inside the `debug` container rather than assuming host behavior.
- If a browser automation flow depends on persistent login, reuse the profile under `docker/home/.box-profile` inside the container.

## Closed Issue Review Workflow

- Prefer `gh` queries that extract public links first, then inspect the linked public PRs or commits to understand the actual patch.
- If logs are required, download them with the `debug` container workflow above; do not copy Box paths, private issue numbers, operator names, or venue-specific internal notes into this file.
- When summarizing a past incident, record three things only: the generalized symptom, the public patch, and the first logs/topics/config to inspect when it happens again.

## Codex Autonomy Triage

- Goal: classify each issue as either `codex_pr_ok` or `human_validation_required`.
- Default rule: if the answer is ambiguous, classify it as `human_validation_required`.
- Primary criterion: the question is not "can Codex edit the code", but "can the fix be validated without a developer or operator checking real robot behavior".

### `codex_pr_ok`

- Use this only when all of the following are true:
  - The failure is deterministic and host-observable: build failure, startup crash, exception, missing dependency, bad script behavior, wrong file lookup, broken API wiring, or logging/debug tooling issue.
  - The success condition is machine-checkable by CI, local command output, unit/integration test, or static inspection.
  - The change does not alter robot motion semantics, localization behavior, perception thresholds, hardware timing, or site-specific map behavior.
  - The change does not require physical devices, phones, sensors, cloud credentials, remote.it, or field environment access to know whether it is correct.
  - A wrong fix would not silently create unsafe motion or degraded navigation in the field.
- Typical examples from past issues:
  - `bake-docker.sh` local registry detection fixes.
  - Docker or pip index fixes for image builds.
  - `rviz2` startup wait workaround.
  - Camera bag throttling and debug-data size reduction.
  - Null-checks, initialization-order fixes, and feature-plugin exceptions where the acceptance criterion is "the exception no longer occurs".

### `human_validation_required`

- Use this if any of the following are true:
  - The issue is about collision, stopping distance, obstacle avoidance, people avoidance, speed control, planner behavior, elevator behavior, crosswalk behavior, or any other robot motion outcome.
  - The issue is about localization drift, wrong floor selection, GNSS/RTK recovery, scan matching, pressure-based floor changes, or map/site parameter tuning.
  - The issue is about perception quality or timing: people detection, tracking merge/split behavior, stop reasons, announcement timing, read timing, or threshold tuning.
  - The issue depends on field logs or operator reports describing behavior such as "too close", "did not say", "sometimes", "random", "unstable", "drifts", or "works better/worse".
  - The proposed fix changes thresholds, filters, topics, footprints, costmap layers, site data, or environment variables that affect runtime navigation behavior.
  - The issue spans multiple devices or services and needs end-to-end workflow confirmation, even if it does not directly change motion.
  - The acceptance criterion is behavioral rather than binary, for example "more stable", "safer", "earlier", "less random", or "better timing".
- Typical examples from past issues:
  - `lidar_speed`, local costmap, or footprint changes.
  - SignalPOI and stop-reason timing fixes.
  - `global_localizer`, `use_nav_sat`, or altitude-manager tuning.
  - Facility `lookup_dist` changes.
  - People-ahead announcement timing and people tracking threshold changes.

### Automatic Heuristics

- Strong `codex_pr_ok` signals in issue text:
  - `build fails`, `image build`, `pip`, `docker`, `registry`, `script`, `exception`, `AttributeError`, `Null`, `startup crash`, `rviz2 crash`, `bag too large`, `log throttling`.
- Strong `human_validation_required` signals in issue text:
  - `collision`, `bump`, `did not stop`, `too close`, `avoid`, `drift`, `wrong floor`, `random localization`, `did not say`, `announcement timing`, `people detection`, `tracking`, `threshold`, `GNSS`, `RTK`, `elevator`, `crosswalk`, `map parameter`.
- Strong `human_validation_required` signals in touched files:
  - `nav2_params.yaml`, `mf_localization/*`, `cabot_ui/src/stop_reasoner*`, `social_navigation.py`, `lidar_speed_control_node.cpp`, `footprint_publisher*`, `cabot_site_*/maps/*`, driver or sensor nodes.
- Strong `codex_pr_ok` signals in touched files:
  - `Dockerfile`, `docker-compose*.yaml`, `bake-docker.sh`, host-side scripts, logging/debug tools, test-only code, UI/backend code whose acceptance criterion is API or rendering correctness.

### Mixed Issues

- If one issue contains both host-side bugs and runtime behavior tuning, split mentally into subproblems.
- Codex may still prepare a draft patch for the host-side part, but the issue classification remains `human_validation_required` unless every shipped change is verifiable without human or robot intervention.

## Sanitized Fix Patterns

- Localization watchdog: intermittent drift can start before any obvious map jump is visible. [cabot-navigation#255](https://github.com/CMU-cabot/cabot-navigation/pull/255) added scan-match and GNSS constraint monitoring in `multi_floor_manager`, and [cabot#276](https://github.com/CMU-cabot/cabot/pull/276) pulled it into the meta-repo. Check `/localize_status`, localization diagnostics, and whether scan/GNSS constraints stop updating before the robot visibly diverges.
- SignalPOI baseline: crosswalk behavior is split across navigation, app-server, and test-site data. [cabot-navigation#238](https://github.com/CMU-cabot/cabot-navigation/pull/238) added SignalPOI logic, stop-reason integration, and announcements; [cabot-app-server#58](https://github.com/CMU-cabot/cabot-app-server/pull/58) publishes signal status to `/signal_response_intersection_status`; [cabot_sites_test#15](https://github.com/CMU-cabot/cabot_sites_test/pull/15) provides a reproducible intersection test site. When debugging signal behavior, check `/cabot/signal_state`, `/signal_response_intersection_status`, remaining-time logic, and whether the issue reproduces in the test site.
- Signal stale-state and proximity gating: signal input should only affect stop logic near the relevant crosswalk and should age out cleanly. [cabot-navigation#263](https://github.com/CMU-cabot/cabot-navigation/pull/263) gated SignalPOI by projected distance and lateral offset, and clears stale signal-derived stop reasons after a timeout. If the robot stops or restarts incorrectly near an intersection, inspect signal message timestamps and POI gating distances before changing thresholds.
- Signal start-stop oscillation: a robot can start on green and stop again immediately if the grace window is too short or if stopped-state thresholds are too high. [cabot-navigation#260](https://github.com/CMU-cabot/cabot-navigation/pull/260) and [53d98b4](https://github.com/CMU-cabot/cabot-navigation/commit/53d98b43af997f97ee7d5b239f70a3682ad62545) reduced the stop velocity thresholds and kept the green state briefly after motion starts. Check velocity around the green transition and whether `NOT_STOPPED` is immediately overwritten by a stopped-under-threshold state.
- People-ahead announcement timing: delayed or missing "person ahead" announcements can come from stop-reason filtering rather than the detector itself. [2878736](https://github.com/CMU-cabot/cabot-navigation/commit/2878736cf1fbdbf00cacd5b9c455d7a41c4db9b5) switched the stop-reason path to `/cabot/social_distance_speed`, shortened the filters and event intervals, and only treats a zero minimum as a blocking person. If the announcement timing feels wrong, inspect `/cabot/social_distance_speed` and stop-reason timing before retuning the detector.
- Collision prevention stack: obstacle contact can persist even when point clouds are visible if multiple safety layers are slightly wrong at once. [3f2052f](https://github.com/CMU-cabot/cabot-navigation/commit/3f2052fe1a7a81860168a775eb8140d44800a4b6) lets `lidar_speed` reach 0 and clamps negative stopping distance, [cabot-navigation#240](https://github.com/CMU-cabot/cabot-navigation/pull/240) switches the local low obstacle layer to `ObstacleLayer` and uses `/livox_scan` for local avoidance, and [cabot-common#31](https://github.com/CMU-cabot/cabot-common/pull/31) reshapes `footprint2` so its inscribed radius better matches the real body. If the robot brushes obstacles while "seeing" them, inspect safety speed limit, local costmap source topic, and footprint geometry together.
- Elevator exit and short-path robustness: elevator incidents were not only map issues; some were BT/path edge cases. [cabot-navigation#266](https://github.com/CMU-cabot/cabot-navigation/pull/266) hardened BT parameter save/restore when blackboard state is missing, and [cabot-navigation#267](https://github.com/CMU-cabot/cabot-navigation/pull/267) ignores null or single-pose paths instead of crashing. During elevator exit failures, inspect BT logs, parameter-restore logs, and path length before changing map data.
- Floor-transition task initialization: an async goto-floor task could fail before its rate object existed. [cabot-navigation#269](https://github.com/CMU-cabot/cabot-navigation/pull/269) initializes `rate` before scheduling the task. If a floor transfer aborts immediately with an attribute error, inspect `navigation.py` exceptions before blaming BLE or the app.
- Indoor GNSS/NTRIP diagnostics: facility Wi-Fi or indoor-only runs can produce misleading RTCM timeout errors even when GNSS is intentionally not part of localization. [ntrip_client 20d3650](https://github.com/CMU-cabot/ntrip_client/commit/20d3650bef85ae63863dcbb9f53ff54e50ac0a3c), [cabot-navigation#256](https://github.com/CMU-cabot/cabot-navigation/pull/256), and [cabot#277](https://github.com/CMU-cabot/cabot/pull/277) made RTCM timeout severity configurable and suppressible when GNSS is unused. Separate true connectivity failures from expected indoor behavior before treating NTRIP warnings as root cause.
- Chinese locale key-read bug: some speech failures were locale normalization bugs rather than missing translations. [cabot-navigation#219](https://github.com/CMU-cabot/cabot-navigation/pull/219) maps runtime `zh` to `zh-CN` and renames the translation file accordingly. If speech reads message keys after a language switch, verify the runtime locale string first.
- Facility lookup distance is site data, not a fixed constant: [cabot-navigation#248](https://github.com/CMU-cabot/cabot-navigation/pull/248) added `hulop_content.lookup_dist`, so outdoor or open-space read timing can now be controlled per facility instead of assuming a hard-coded 5 m lookup and 8 m nearby threshold. If read timing feels too early or too late, inspect the facility metadata before patching navigation code.
- Feature-plugin parameter updates: runtime setting changes can fail inside the feature plugin before any ROS parameter update is attempted. [f5a4ed6](https://github.com/CMU-cabot/cabot-navigation/commit/f5a4ed6b110c407c86fa0b3c95e67c173204de00) added a logger and `ParamManager` to the feature plugin, made parameter-client lookup return `None` safely, and loads `feature` before `navigation`. If handle-side or touch-mode changes do not apply, inspect plugin startup errors and parameter-client availability rather than only the target node.
- Robot-specific diagnostics and IMU covariance: some debugging sessions were blocked by poor diagnostics rather than the original fault. [a65d718](https://github.com/CMU-cabot/cabot-navigation/commit/a65d718ef9db604a9374c8359420aae753f4049f) and [ab0b6d1](https://github.com/CMU-cabot/cabot-navigation/commit/ab0b6d1bbd09a2ecbc8fc2bcb5cadba78a86783f) added and fixed `cabot3-k1` diagnostic aggregation, and [ac686e3](https://github.com/CMU-cabot/cabot-drivers/commit/ac686e371872280cb1b68de11bf902fc534b0f5e) initializes IMU orientation covariance. Before chasing planner or localization behavior on new hardware, verify that the robot-specific diagnostic config is loaded and that IMU covariance is nonzero.
- Camera logging for long debug sessions: full-rate multi-camera bagging can grow too large to keep around. [cabot-common#29](https://github.com/CMU-cabot/cabot-common/pull/29), [cabot-navigation#227](https://github.com/CMU-cabot/cabot-navigation/pull/227), and [cabot#261](https://github.com/CMU-cabot/cabot/pull/261) added timestamp-based image throttling and `CABOT_ROSBAG_CAMERA_THROTTLE_HZ` for recording synchronized reduced-rate camera topics. When logs need image context but disk budget is tight, prefer throttled image recording to ad hoc topic drops.
- GUI startup race: [b56227f](https://github.com/CMU-cabot/cabot-navigation/commit/b56227fccffcd6f8e5476acf8ac9df30594e3385) adds `-w 1` before launching `gui` to prevent `rviz2` from crashing immediately after startup. If `rviz2` falls over only during launch, try a startup delay before digging into RViz plugin state.
- Build-time environment quirks: [cabot-navigation#244](https://github.com/CMU-cabot/cabot-navigation/pull/244) adds `PIP_EXTRA_INDEX_URL=https://pypi.org/simple` so the embedding image can recover when the custom index is incomplete, and [cabot#281](https://github.com/CMU-cabot/cabot/pull/281) makes `bake-docker.sh` match the local registry container more strictly by name, network, and published port. If image builds fail only on some hosts or fail late during push, check package-index fallback and local-registry discovery before changing the broader build flow.

## General cabot_sites Patterns

- Site-level altitude tuning: false floor transitions are often a `cabot_sites` configuration problem before they are a core localization bug. Check pressure standard deviation, altitude queue behavior, floor-height assumptions, and any altitude-manager thresholds that decide when vertical movement starts.
- Site-level scan-match recovery tuning: if localization drifts within one floor and only recovers after manual relocalization, inspect the site parameters that control how easily tracking mode creates scan-match constraints. Lowering the tracking threshold can improve recovery, but it should be paired with narrow-space guards so elevators and other tight spaces do not introduce bad matches.
- Site-level global initialization and GNSS assistance: outdoor or mixed indoor/outdoor sites may need more than ordinary scan matching for initial pose and long-range recovery. Check whether the site provides the required global-localization assets, whether RTK or Wi-Fi assistance is enabled where intended, and whether the ENU frame, rotation, and related `use_nav_sat` settings are consistent with how the map was created.

## Improvement Ideas

- Consider storing the Issue text together with the downloaded logs for later debugging context.
