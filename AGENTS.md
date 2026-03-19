# AGENTS.md

## Repository Shape and Dependency Setup

- This workspace is a meta-repository that coordinates multiple component repositories.
- Nested repository membership and checkout are managed by the dependency setup files used by `./setup-dependency.sh`.
- Important modes:
  - `./setup-dependency.sh -r`
    - release-oriented dependency setup
  - `./setup-dependency.sh -d`
    - common development setup
  - `./setup-dependency.sh -d -o`
    - development setup with overrides
- Some changes must be committed inside a nested repository, not only at the top level.
- Before committing, always check both:
  - `git status --short`
  - `git -C <nested-repo> status --short`
- Running `./setup-dependency.sh` without options is usually for server-side or build-only environments, not normal development work.

### Updating Repositories

- A common recursive update command is:
  - `vcs -n pull`
- This can affect many nested repositories at once. Use it carefully.
- If the expected impact is broad or risky, prefer pulling individual repositories with `git pull` instead of updating everything recursively.

## Working Rules

- Prefer small, targeted changes. Do not reformat unrelated files.
- Do not overwrite user changes in a dirty worktree.
- If a task touches code under `cabot-navigation/`, inspect that repository's Git state separately.
- When a change spans both top-level files and a nested repository, expect two commits.
- Use `rg` for search and `docker compose` for container workflows.
- For any bag file fix, always use `./tools/fix_bag.sh -f <bag>`.

## Docker and Compose

- Main entrypoints are the top-level compose files such as:
  - `docker-compose.yaml`
  - `docker-compose-common.yaml`
  - `docker-compose-*.yaml`
- Container image selection follows `CABOT_LAUNCH_IMAGE_TAG` from the environment or `.env`.
- The standard development startup pattern is:
  - `docker compose --profile dev up -d <service>`
- The standard image build pattern is:
  - `docker compose --profile dev build <service>`
- You can download a matching set of tagged images with:
  - `./manage-pkg.sh -p <tag>`

## Build Modes

- There are two distinct build categories:
  - Docker container image builds
  - ROS workspace builds
- Runtime modes are also split into:
  - dev
  - prod
- During development, prefer `dev`.
- In `dev`, the system uses locally built workspace artifacts.
- In `prod`, the system uses binaries already installed in the container image.
- For container images:
  - Pulling prebuilt images from `docker.io` is the simplest path when no local Docker context changes are needed.
  - If Docker contexts, Dockerfiles, or image-layer dependencies changed, build locally instead.
  - Local image build entrypoint:
    - `./bake-docker.sh -i <container>`
- For workspace builds:
  - Host ROS 2 build:
    - `./build-workspace.sh -o`
  - Development-container build:
    - `./build-workspace.sh -w`
  - The container-based workspace build writes persistent build artifacts under `docker/home`.

## Debug Workflow

- Debug and Playwright-specific workflow is documented in `AGENTS-DEBUG.md`.
- If the user provides a GitHub Issue URL and asks to debug or triage that issue, prefer using `./cabot_debug/script/codex_issue_debug.sh <github-issue-url>` from the repository root unless they explicitly ask for a manual workflow.

## Validation

- For compose changes, prefer validating with:
  - `docker compose -f docker-compose.yaml config`
- Before creating or updating a PR with code changes, run unit tests in each repository where changes were made and confirm they pass.
  - Standard path: `./launch.sh -u -- -a`
  - Some repositories use: `./unittest.sh`

## Commit Guidance

- Use signed-off commits for every commit with `git commit -s` or `git commit --amend -s`.
- When the task changes both top-level and nested repositories, commit nested repositories first, then the top-level repository if needed.
- Write commit messages that describe the operational change, not just the edited files.
