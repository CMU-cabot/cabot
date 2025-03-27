#!/bin/bash

trap ctrl_c INT
function ctrl_c() {
    echo "Stopping all processes..."
    for pid in "${pids[@]}"; do
        kill -SIGINT "$pid"
    done
    wait
    exit 0
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

pids=()
file=
while getopts ":f:" opt; do
  case $opt in
    f) file="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
       exit 1
    ;;
  esac
done
shift $((OPTIND -1))

if [ -z "$file" ]; then
  echo "Usage: $0 -f <filename>"
  exit 1
fi

pushd $scriptdir/..
CABOT_MODEL=cabot3-k4 docker compose run -T --rm driver /launch.sh power &
pids+=($!)
popd

source $scriptdir/../host_ws/install/setup.bash

ros2 bag record /battery_states &
pids+=($!)

sleep 5

canplayer -t -I $file -v


echo "Stopping all processes..."
for pid in "${pids[@]}"; do
    kill -SIGINT "$pid"
done
wait
echo "All processes stopped."