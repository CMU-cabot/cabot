#!/bin/bash

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "Test terminated"
    exit 0
}

function err {
    >&2 red "[ERROR] "$@
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function help()
{
    echo "Usage:"
    echo "-h          show this help"
    echo "-d          log directory"
    echo "-b          build host_ws"
}

blue "Start runnint tests"

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
source $scriptdir/../.env

test_dir=$(find $scriptdir/../cabot_sites -name $CABOT_SITE | head -n 1)
tests=$test_dir/test/tests.yaml

log_dir=
build_host_ws=

while getopts "hd:b" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
	    log_dir=$OPTARG
            ;;
	b)
	    build_host_ws=1
	    ;;
    esac
done

blue "testing with $CABOT_SITE"

if [[ $build_host_ws -eq 1 ]]; then
    pushd $scriptdir/../host_ws/
    colcon build --symlink-install
    popd
fi

source $scriptdir/../host_ws/install/setup.bash

ROS_LOG_DIR=$log_dir ros2 run cabot_debug run_test.py -f $tests
result=$?

echo "Test has completed"
exit $result
