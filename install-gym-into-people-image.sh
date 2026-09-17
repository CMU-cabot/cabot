#!/usr/bin/env bash

# Make the upstream cabot-people image able to run lidar_process, in place.
#
# The upstream JetPack 6 people image has everything lidar_process needs except
# two Python packages:
#
# 1. gym. rl_server.py imports crowd_attn_rl unconditionally for every
#    CABOT_CONTROLLER mode, and that does `import gym`. Upstream does not install
#    it because upstream has no lidar_process.
#
# 2. An old enough numpy. lidar_process and the crowdattn / group_rl code it
#    pulls in use `np.bool` (13 places), which numpy 1.24 removed; the image has
#    1.24.1, so rl_server dies in CrowdAttnRL._set_spaces with
#    "module 'numpy' has no attribute 'bool'". The JetPack 5 EXPO image pinned
#    numpy 1.23.1 for the same reason. 1.23.x shares the C-API version (0x10)
#    with 1.24.x, so everything compiled against 1.24.1 in the image (cv2, torch,
#    mmcv, mmdeploy, scipy, sklearn) keeps working; that was checked on the
#    JetPack 6.0 image before choosing this.
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
# last numpy that still has np.bool (as a deprecated alias of bool)
NUMPY_VERSION=${NUMPY_VERSION:-1.23.5}

red()  { echo -e "\033[31m$*\033[0m"; }
blue() { echo -e "\033[36m$*\033[0m"; }

# What the patched image has to be able to do. Import failures of the compiled
# packages are what a numpy ABI mismatch looks like, so they are checked
# explicitly rather than trusted.
read -r -d '' CHECK_PY <<'EOF'
import sys, warnings
warnings.filterwarnings("ignore")
failed = []
def check(name, fn):
    try:
        print("  ok   %-10s %s" % (name, fn()))
    except Exception as e:
        print("  FAIL %-10s %s: %s" % (name, type(e).__name__, str(e)[:160]))
        failed.append(name)

import numpy as np
check("numpy", lambda: "%s, np.bool is bool: %s" % (np.__version__, np.bool is bool))
if not getattr(np, "bool", None) is bool:
    failed.append("np.bool")

def _gym():
    import gym
    b = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(1, 7), dtype=np.float32)
    gym.spaces.Dict({"robot_node": b})
    gym.spaces.Box(low=-np.inf, high=np.inf, shape=(20,), dtype=np.bool)
    return gym.__version__
check("gym", _gym)

def _cv2():
    import cv2
    return "%s cuda devices=%d" % (cv2.__version__, cv2.cuda.getCudaEnabledDeviceCount())
check("cv2", _cv2)

def _torch():
    import torch
    torch.from_numpy(np.zeros(3, dtype=np.bool))
    return "%s cuda=%s" % (torch.__version__, torch.cuda.is_available())
check("torch", _torch)

for m in ["scipy", "sklearn", "mmcv", "mmdeploy"]:
    check(m, lambda m=m: __import__(m).__version__)

sys.exit(1 if failed else 0)
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
def v(p):
    try: return md.version(p)
    except md.PackageNotFoundError: return 'none'
print(v('gym'), v('numpy'))" 2>/dev/null)
    have_gym=${have% *}
    have_numpy=${have#* }
    if [[ "$have_gym" == "$GYM_VERSION" ]] && [[ "$have_numpy" == "$NUMPY_VERSION" ]]; then
        echo "  gym $have_gym, numpy $have_numpy -- nothing to do"
        continue
    fi
    echo "  have gym ${have_gym:-?}, numpy ${have_numpy:-?}; want gym $GYM_VERSION, numpy $NUMPY_VERSION"

    # The people images run their entrypoint as root: it does
    # `usermod -u $HOST_UID developer` and then `exec gosu developer`, and gosu is
    # not setuid, so both need root. Whatever user the base image ends on has to
    # be put back, or the entrypoint dies with "user developer is currently used
    # by process 1" (usermod exit code 8) and set -e takes the container with it.
    orig_user=$(docker image inspect --format '{{.Config.User}}' "$ref")
    restore_user=${orig_user:-root}

    # --no-deps for both. gym declares scipy, six, pyglet and cloudpickle~=1.2.0,
    # but `import gym` only needs numpy and six, both already in the image;
    # resolving the rest would pull cloudpickle 1.2.2, which predates Python 3.9.
    # numpy has no dependencies, and --no-deps keeps pip from touching anything
    # that declares a numpy requirement.
    read -r -d '' DOCKERFILE <<EOF
ARG FROM_IMAGE=cmucal/cabot-people:${TAG}
FROM \${FROM_IMAGE}
USER root
RUN pip3 install --no-cache-dir --no-deps numpy==${NUMPY_VERSION} gym==${GYM_VERSION} six
USER ${restore_user}
EOF

    echo "  building $tmp (restoring USER '${restore_user}')"
    if ! echo "$DOCKERFILE" | docker build -q --build-arg FROM_IMAGE="$ref" -t "$tmp" -f - . > /dev/null; then
        red "  build failed, $ref left untouched"
        status=1
        continue
    fi

    ok=1
    if ! docker run --rm --entrypoint python3 "$tmp" -c "$CHECK_PY"; then
        ok=0
    fi

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
