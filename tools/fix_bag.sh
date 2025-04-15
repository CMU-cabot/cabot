#!/bin/bash

function help {
    echo "Usage: $0 [<options>] <bag_file_path>"
    echo "-h          show this help"
    echo "-c          check if bag is valid"
    echo "-f          fix bag"
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

# change directory to where this script exists
pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

cd $scriptdir/../

source ./host_ws/install/setup.bash

check=
fix=
while getopts "hcf" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
    c)
        check=1
        ;;
	f)
	    fix=1
	    ;;
    esac
done
shift $((OPTIND-1))

bag=$1
if [[ -z $bag ]]; then
    help
    exit 1
fi

if [[ $check -eq 1 ]]; then
    if [[ -e $bag/metadata.yaml ]]; then
        blue "Bag file $bag looks valid"
        ros2 bag info $bag
        exit 0
    else
        red "Bag file $bag looks not valid"
    fi
fi
if [[ $fix -eq 1 ]]; then
    if ros2 bag info $bag; then
        blue "Bag file $bag looks valid, checking compression options"
        python3 $scriptdir/fix_bag.py $bag
        ros2 bag info $bag
        exit 0
    else
        blue "Reindexing file $bag"
        ros2 bag reindex $bag sqlite3
        blue "Checking compression options"
        python3 $scriptdir/fix_bag.py $bag
    fi
fi
