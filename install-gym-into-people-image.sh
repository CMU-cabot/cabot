#!/usr/bin/env bash

# Add gym to the cabot-people image, in place.
#
# The upstream JetPack 6 people image already has everything lidar_process needs
# except gym: rl_server.py imports crowd_attn_rl unconditionally for every
# CABOT_CONTROLLER mode, and that does `import gym`. Upstream does not install it
# because upstream has no lidar_process.
#
# This adds that one module and re-tags the result as the same tag, so .env and
# docker compose need no change. Re-running `./manage-pkg.sh -p <tag>` pulls the
# upstream image again and undoes this; just run this script again afterwards.
# It is idempotent, so re-running it when nothing is missing costs nothing.
#
# Usage:
#   ./install-gym-into-people-image.sh [tag] [image ...]
#
#   ./install-gym-into-people-image.sh
#   ./install-gym-into-people-image.sh v2.5.1-beta29-jetpack60
#   ./install-gym-into-people-image.sh v2.5.1-beta29-jetpack60 cmucal/cabot-people

set -u

# tag this branch is meant to run against
DEFAULT_TAG=v2.5.1-beta29-jetpack60

TAG=${1:-$DEFAULT_TAG}
shift || true
if [[ $# -gt 0 ]]; then
    IMAGES=("$@")
else
    # cabot-people-nuc is deliberately not here: it is published for amd64 only
    # (it is the Intel NUC variant), so there is nothing to patch on a Jetson.
    IMAGES=(cmucal/cabot-people cmucal/cabot-people-framos)
fi

# gym 0.15.7 is what the JetPack 5 EXPO image ran: openai/baselines pinned it
# there with 'gym>=0.15.4, <0.16.0'. Verified to import on Python 3.10 (JetPack 6).
GYM_VERSION=${GYM_VERSION:-0.15.7}

red()  { echo -e "\033[31m$*\033[0m"; }
blue() { echo -e "\033[36m$*\033[0m"; }

# --no-deps is deliberate. gym declares scipy, six, pyglet and cloudpickle~=1.2.0,
# but `import gym` only needs numpy and six at module level, and both are already
# in the image. Resolving the declared deps would pull cloudpickle 1.2.2, which
# predates Python 3.9, and would risk moving numpy out from under the mmcv /
# mmdeploy builds in this image.
#
# USER root is required: the image ends as USER developer, and cabot's
# docker-compose-common.yaml bind-mounts ./docker/home over /home/developer for
# people-dev, so anything pip puts in ~/.local disappears at runtime.
read -r -d '' DOCKERFILE <<EOF
ARG FROM_IMAGE=cmucal/cabot-people:${TAG}
FROM \${FROM_IMAGE}
USER root
RUN pip3 install --no-cache-dir --no-deps gym==${GYM_VERSION} six
USER developer
EOF

status=0
for image in "${IMAGES[@]}"; do
    ref="${image}:${TAG}"
    echo
    blue "=== $ref"

    if ! docker image inspect "$ref" > /dev/null 2>&1; then
        red "  image not found, skipping. Pull it first (./manage-pkg.sh -p $TAG)"
        continue
    fi

    installed=$(docker run --rm --entrypoint python3 "$ref" \
                -c "import gym; print(gym.__version__)" 2>/dev/null)
    if [[ "$installed" == "$GYM_VERSION" ]]; then
        echo "  gym $installed is already installed, nothing to do"
        continue
    elif [[ -n "$installed" ]]; then
        red "  gym $installed is installed but $GYM_VERSION was requested, rebuilding"
    fi

    before=$(docker image inspect --format '{{.Id}}' "$ref")

    echo "  installing gym==$GYM_VERSION"
    if ! echo "$DOCKERFILE" | docker build --build-arg FROM_IMAGE="$ref" -t "$ref" -f - . ; then
        red "  build failed"
        status=1
        continue
    fi

    after=$(docker image inspect --format '{{.Id}}' "$ref")
    echo "  ${before:7:12} -> ${after:7:12}"

    # the point of the exercise: rl_server must be able to import gym and build
    # the observation / action spaces crowd_attn_rl.py uses
    if docker run --rm --entrypoint python3 "$ref" -c "
import gym, numpy as np
b = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(1, 7), dtype=np.float32)
gym.spaces.Dict({'robot_node': b})
print('  verified: gym', gym.__version__, '/ numpy', np.__version__)
" 2>&1 | grep -v UserWarning | grep -v warnings.warn; then
        :
    else
        red "  verification failed"
        status=1
    fi
done

echo
if [[ $status -eq 0 ]]; then
    blue "done"
else
    red "finished with errors"
fi
exit $status
