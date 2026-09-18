#!/usr/bin/env bash

# Add gym to the cabot-people image, in place.
#
# The upstream JetPack 6 people image has everything lidar_process needs except
# gym: rl_server.py imports crowd_attn_rl unconditionally for every
# CABOT_CONTROLLER mode, and that does `import gym`. Upstream does not install it
# because upstream has no lidar_process.
#
# numpy is deliberately left alone. lidar_process used to need numpy < 1.24 for
# np.bool, and an earlier version of this script pinned 1.23.5 the way the
# JetPack 5 EXPO image did. That is fixed in the source now (crowd_attn_rl.py
# spells the dtype as the builtin bool, which is all np.bool ever was), so the
# image keeps whatever numpy it shipped with. That matters for JetPack 6.2: its
# base image carries a newer numpy, and downgrading across a C-API generation
# would break everything compiled against it -- cv2, torch, mmcv, mmdeploy.
#
# The result is re-tagged as the same tag, so .env and docker compose need no
# change. Re-running `./manage-pkg.sh -p <tag>` pulls the upstream image again
# and undoes this; just run this script again afterwards. It is idempotent.
#
# Nothing is overwritten unless the patched image passes the checks below: it is
# built under a temporary tag first and only then moved onto the real one.
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

# What the patched image has to be able to do: build the observation and action
# spaces CrowdAttnRL._set_spaces builds, including the bool one that used to be
# spelled np.bool.
read -r -d '' CHECK_PY <<'EOF'
import sys, warnings
warnings.filterwarnings("ignore")
import numpy as np
import gym
b = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(1, 7), dtype=np.float32)
gym.spaces.Dict({"robot_node": b})
gym.spaces.Box(low=-np.inf, high=np.inf, shape=(20,), dtype=bool)
print("  ok   gym %s, numpy %s" % (gym.__version__, np.__version__))
EOF

status=0
for image in "${IMAGES[@]}"; do
    ref="${image}:${TAG}"
    tmp="${image}:${TAG}-patching"
    echo
    blue "=== $ref"

    if ! docker image inspect "$ref" > /dev/null 2>&1; then
        red "  image not found, skipping. Pull it first (./manage-pkg.sh -p $TAG)"
        continue
    fi

    have=$(docker run --rm --entrypoint python3 "$ref" -c "
import importlib.metadata as md
try: print(md.version('gym'))
except md.PackageNotFoundError: print('none')" 2>/dev/null)
    if [[ "$have" == "$GYM_VERSION" ]]; then
        echo "  gym $have is already installed, nothing to do"
        continue
    fi
    echo "  have gym ${have:-?}; want $GYM_VERSION"

    # The people images run their entrypoint as root: it does
    # `usermod -u $HOST_UID developer` and then `exec gosu developer`, and gosu is
    # not setuid, so both need root. Whatever user the base image ends on has to
    # be put back, or the entrypoint dies with "user developer is currently used
    # by process 1" (usermod exit code 8) and set -e takes the container with it.
    orig_user=$(docker image inspect --format '{{.Config.User}}' "$ref")
    restore_user=${orig_user:-root}

    # --no-deps is deliberate. gym declares scipy, six, pyglet and
    # cloudpickle~=1.2.0, but `import gym` only needs numpy and six at module
    # level, and both are already in the image. Resolving the declared deps would
    # pull cloudpickle 1.2.2, which predates Python 3.9, and would risk moving
    # numpy out from under the mmcv / mmdeploy builds in this image.
    read -r -d '' DOCKERFILE <<EOF
ARG FROM_IMAGE=cmucal/cabot-people:${TAG}
FROM \${FROM_IMAGE}
USER root
RUN pip3 install --no-cache-dir --no-deps gym==${GYM_VERSION} six
USER ${restore_user}
EOF

    echo "  building $tmp (restoring USER '${restore_user}')"
    if ! echo "$DOCKERFILE" | docker build -q --build-arg FROM_IMAGE="$ref" -t "$tmp" -f - . > /dev/null; then
        red "  build failed, $ref left untouched"
        status=1
        continue
    fi

    ok=1
    docker run --rm --entrypoint python3 "$tmp" -c "$CHECK_PY" || ok=0

    # the entrypoint must still be able to do its uid dance, which is what breaks
    # if USER is left as developer
    if docker run --rm -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" "$tmp" \
       id 2>&1 | grep -q "uid=$(id -u)"; then
        echo "  ok   entrypoint switches to uid $(id -u)"
    else
        echo "  FAIL entrypoint does not reach uid $(id -u); HOST_UID handling is broken"
        ok=0
    fi

    if [[ $ok -eq 0 ]]; then
        red "  checks failed, $ref left untouched"
        docker rmi "$tmp" > /dev/null 2>&1
        status=1
        continue
    fi

    before=$(docker image inspect --format '{{.Id}}' "$ref")
    docker tag "$tmp" "$ref"
    docker rmi "$tmp" > /dev/null 2>&1
    after=$(docker image inspect --format '{{.Id}}' "$ref")
    echo "  ${before:7:12} -> ${after:7:12}"
done

echo
if [[ $status -eq 0 ]]; then
    blue "done"
else
    red "finished with errors"
fi
exit $status
