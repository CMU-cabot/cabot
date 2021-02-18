#!/bin/bash

# Copyright (c) 2021  IBM Corporation
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


## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    ## killing all nodes
    # comment out because this will kill remote simulator nodes
    # echo "killing all ros nodes..."
    # rosnode kill -a

    for pid in ${pids[@]}; do
    	echo "killing $pid..."
    	kill -s 2 $pid
    done

    ## we need to wait gazebo is terminated
    rlc=0
    while [ `ps -A | grep roslaunch | wc -l` -ne 0 ];
    do
	snore 1
	ps -A
	echo -ne "waiting process is completely terminated ($rlc)"\\r
	rlc=$((rlc+1))
    done
    
    echo \\n
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

### default variables

## debug
#minimum=0
debug=0
command=''
commandpost='&'

# for localization
points2_topic='/velodyne_points'
imu_topic='/cabot/imu/data'
beacons_topic='/wireless/beacons'
odom_topic='/cabot/odom'
publish_current_rate=0

gazebo=0

show_rviz=1
site=

# for navigation
navigation=0
map_server=0
robot='cabot2-e2'
robot_desc='cabot2-e2'
with_human=1
gplanner='base_global_planner:=navfn/NavfnROS'
lplanner='base_local_planner:=dwa_local_planner/DWAPlannerROS'
cmd_vel_topic='/cabot/raw_cmd_vel'

# optional
use_cvi=0

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    echo "ex)"
    echo $0 "-O -T <site_package>"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug"
    echo "-m <map file>            specify a map file"
    echo "-n <anchor file>         specify a anchor file, use map file if not specified"
    echo "-w <world file>          specify a world file"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-O                       performance (no rviz)"
    echo "-T <site package>        packge name for the robot site"
    echo "-N                       start ROS1 navigation"
    echo "-M                       start multi-floor map server"
    echo "-r <robot>               specify a robot for navigation"
    echo "-f                       use robot footprint without human for navigation"
    echo "-R <rate:float>          set publish_current_rate"
    echo "-C                       run cvi client"
    exit
}

while getopts "hdm:n:w:sOT:NMr:fR:C" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
  d)
	    debug=1
	    command="setsid xterm -e '"
	    commandpost=";read'&"
	    ;;
	m)
	    map=$OPTARG
	    ;;
	n)
	    anchor=$OPTARG
	    ;;
	w)
	    world=$OPTARG
	    ;;
  s)
	    gazebo=1
	    ;;
  O)
	    show_rviz=0
	    ;;
	T)
	    site=$OPTARG
	    ;;
  N)
	    navigation=1
	    ;;
  M)
	    map_server=1
	    ;;
  r)
	    robot=$OPTARG
	    robot_desc=$robot
	    ;;
  f)
	    with_human=0
	    ;;
  R)
            publish_current_rate=$OPTARG
	    ;;
  C)
	    use_cvi=1
	    ;;
  esac
done
shift $((OPTIND-1))

# load site package
if [ "$site" != "" ]; then
    sitedir=`rospack find $site`
    source $sitedir/config/config.sh
    if [ "$map" == "" ] && [ "$world" == "" ]; then
    	echo "Please check config/config.sh in site package ($sitedir) to set map and world"
    	exit
    fi
else
    if [ "$map" == "" ]; then
    	echo "-T <site> or -m <map> should be specified"
    	exit
    fi
    if [ $gazebo -eq 1 ] && [ "$world" == "" ]; then
    	echo "-T <site> or -w <world> should be specified"
    	exit
    fi
fi


## debug output
echo "Debug         : $debug ($command, $commandpost)"
echo "World         : $world"
echo "Map           : $map"
echo "Anchor        : $anchor"
echo "Simulation    : $gazebo"
echo "Navigation    : $navigation"
echo "Map server    : $map_server"
echo "With human    : $with_human"
echo "Robot         : $robot"
echo "Global planner: $gplanner"
echo "Local planner : $lplanner"

# roscore
#rosnode list
#if [ $? -eq 1 ]; then
#    eval "$command roscore $commandpost"
#    pids+=($!)
#fi

rosnode list
test=$?
while [ $test -eq 1 ]; do
    snore 0.1
    c=$((c+1))
    echo "wait roscore" $c
    rosnode list
    test=$?
done

### For GAZEBO simulation, run wireless simulator with gazebo
### For physical robots, run wireless scanner separately
if [ $gazebo -eq 1 ]; then
  if [ "$wireless_config" == "" ]; then
    echo "does not launch ble_rss_simulator (world_config was not found)."
  else
    echo "launch gazebo helpers (ble_rss_simulator, floor_transition)"
    wireless_config=$(realpath $wireless_config)
    eval "$command roslaunch mf_localization_gazebo gazebo_helper.launch \
                    beacons_topic:=$beacons_topic \
                    world_config_file:=$wireless_config \
                    $commandpost"

    pids+=($!)
  fi
fi

### launch rviz
if [ $show_rviz -eq 1 ]; then
   echo "launch rviz"
   eval "$command roslaunch mf_localization view_multi_floor.launch $commandpost"
   pids+=($!)
fi

### launch localization
if [ $navigation -eq 0 ]; then
  # run mf_localization only
  launch_file="multi_floor_2d_rss_localization.launch"
  echo "launch $launch_file"
  eval "$command roslaunch mf_localization $launch_file \
                  robot:=$robot_desc \
                  map_config_file:=$map \
                  with_odom_topic:=true \
                  beacons_topic:=$beacons_topic \
                  points2_topic:=$points2_topic \
                  imu_topic:=$imu_topic \
                  odom_topic:=$odom_topic \
		  publish_current_rate:=$publish_current_rate \
                  $commandpost"
  pids+=($!)

  # launch multi-floor map server for visualization
  if [ $map_server -eq 1 ]; then
    echo "launch multi_floor_map_server.launch"
    eval "$command roslaunch mf_localization multi_floor_map_server.launch \
                    map_config_file:=$map \
                    $commandpost"
    pids+=($!)
  fi
else
  # run navigation (mf_localization + planning)
  echo "launch multicart_demo.launch"
  eval "$command roslaunch cabot_mf_localization multicart_demo.launch \
    map_file:=$map \
    beacons_topic:=$beacons_topic \
    points2_topic:=$points2_topic \
    imu_topic:=$imu_topic \
    odom_topic:=$odom_topic \
    publish_current_rate:=$publish_current_rate \
    cmd_vel_topic:=$cmd_vel_topic \
    $lplanner $gplanner \
    robot:=$robot_desc \
    with_human:=$with_human \
    $commandpost"
  pids+=($!)
fi

### launch cvi client
if [ $use_cvi -eq 1 ]; then
   echo "launch cvi client"
   eval "$command roslaunch cvi_client cvi_client.launch $commandpost"
   pids+=($!)
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
