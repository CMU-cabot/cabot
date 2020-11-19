#!/bin/bash

# Copyright (c) 2020 Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

site=""
debug=0
init_x=0
init_y=0
init_a=0
init_z=0
gazebo=0
use_sim_time=false
amcl=1
pid=
show_rviz=true
show_local_rviz=true

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "killing all process"
    kill -s 2 0
    snore 3
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function usage {
    echo "Usage"
    echo "ex)"
    echo $0 "-T"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug (without xterm)"
    echo "-x <initial x>           specify initial position of x"
    echo "-y <initial y>           specify initial position of y"
    echo "-Z                       specify initial position of z"
    echo "-a <initial angle>       specify initial angle (degree)"
    echo "-T <site package>        packge name for the robot site"
    echo "-s                       gazebo simulation"
    echo "-M                       multifloor localization"
    echo "-o                       no local rviz2"
    echo "-O                       no rviz2"
    exit
}


while getopts "hdT:x:y:Z:a:MsoO" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	d)
	    debug=1
	    ;;
	s)
	    gazebo=1
		use_sim_time=true
	    ;;
	T)
	    site=$OPTARG
	    ;;
	x)
	    init_x=$OPTARG
	    ;;
	y)
	    init_y=$OPTARG
	    ;;
	a)
	    inita=`echo "$OPTARG * 3.1415926535 / 180.0" | bc -l`
	    ;;
	Z)
	    init_z=$OPTARG
	    ;;
	M)
	    amcl=0
	    ;;
    o)
        show_local_rviz=false
        ;;
    O)
        show_rviz=false
        ;;
    esac
done
shift $((OPTIND-1))

cd $scriptdir/../../..
if [ $debug -eq 1 ]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
else
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo > /dev/null
fi
if [ $? -ne 0 ]; then
    exit
fi
source $scriptdir/../../../install/setup.bash

if [ "$site" != "" ]; then
    sitedir=`ros2 pkg prefix $site`/share/$site
    echo $sitedir
    source $sitedir/config/config.sh
    if [ "$map" == "" ]; then
	echo "Please check config/config.sh in site package ($sitedir) to set map and world"
	exit
    fi
else
    if [ "$map" == "" ]; then
	echo "-T <site> or -m <map> should be specified"
	exit
    fi
fi

echo "Debug         : $debug"
echo "Simulation    : $gazebo"
echo "Site          : $site"
echo "Init X        : $init_x"
echo "Init Y        : $init_y"
echo "Init A        : $init_a"
echo "Init Z        : $init_z"
echo "map           : $map"
echo "use_amcl      : $amcl"
echo "show rviz     : $show_rviz"
echo "show local rviz : $show_local_rviz"

ros2 launch cabot_navigation2 bringup_launch.py map:=$map use_amcl:=$amcl autostart:=true use_sim_time:=$use_sim_time \
    show_rviz:=$show_rviz show_local_rviz:=$show_local_rviz&

mkdir -p ~/.ros/log/bt_log
ros2 bag record -o ~/.ros/log/bt_log/`date +%F-%H-%M-%S` /behavior_tree_log /evaluation &
ros2 bag record -o ~/.ros/log/bt_log/`date +%F-%H-%M-%S`-local /local/behavior_tree_log /local/evaluation &

while [ 1 -eq 1 ];
do
    snore 1
done


