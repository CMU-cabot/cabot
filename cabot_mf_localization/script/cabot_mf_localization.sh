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
    blue "trap cabot_mf_localization.sh "
    
    for pid in ${pids[@]}; do
	blue "send SIGINT to $pid"
        com="kill -INT -$pid"
        eval $com
    done
    for pid in ${pids[@]}; do
	count=0
        while kill -0 $pid 2> /dev/null; do
	    if [[ $count -eq 3 ]]; then
		blue "escalate to SIGTERM $pid"
		com="kill -TERM -$pid"
		eval $com
	    fi
	    if [[ $count -eq 10 ]]; then
		blue "escalate to SIGKILL $pid"
		com="kill -KILL -$pid"
		eval $com
	    fi
            echo "waiting $0 $pid"
            snore 1
	    count=$((count+1))
        done
    done
    
    exit
}


## todo duplicate code
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
wifi_topic='/esp32/wifi'
odom_topic='/odom_'
pressure_topic='/cabot/pressure'
gnss_fix_topic='/ublox/fix'
gnss_fix_velocity_topic='/ublox/fix_velocity'
publish_current_rate=0

: ${CABOT_GAZEBO:=0}
: ${CABOT_SITE:=}
: ${CABOT_MODEL:=}
: ${CABOT_SHOW_LOC_RVIZ:=0}
: ${CABOT_PRESSURE_AVAILABLE:=0}
: ${CABOT_USE_GNSS:=0}

gazebo=$CABOT_GAZEBO
site=$CABOT_SITE
show_rviz=$CABOT_SHOW_LOC_RVIZ
robot=$CABOT_MODEL
robot_desc=$CABOT_MODEL
# set 0 to the default value so that adding -p means using pressure topic.
pressure_available=$CABOT_PRESSURE_AVAILABLE
use_gnss=$CABOT_USE_GNSS

# for navigation
navigation=0
localization=1
cart_mapping=0
map_server=0
with_human=1
gplanner='base_global_planner:=navfn/NavfnROS'
lplanner='base_local_planner:=dwa_local_planner/DWAPlannerROS'
cmd_vel_topic='/cmd_vel'

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
    echo "-X                       do not start localization"
    echo "-C                       run cartographer mapping"
    echo "-p                       use pressure topic for height change detection"
    echo "-G                       use gnss fix for outdoor localization"
    exit
}

while getopts "hdm:n:w:sOT:NMr:fR:XCpG" arg; do
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
    X)
	localization=0
	;;
    C)
	cart_mapping=1
	;;
    p)
	pressure_available=1
	;;
    G)
	use_gnss=1
	;;
  esac
done
shift $((OPTIND-1))

# load site package
if [ "$site" != "" ]; then
    sitedir=`ros2 pkg prefix $site`/share/$site
    source $sitedir/config/config.sh
    if [ "$map" == "" ] && [ "$world" == "" ]; then
        echo "Please check config/config.sh in site package ($sitedir) to set map and world"
        exit
    fi
else
    if [ $cart_mapping -eq 0 ]; then
	if [ "$map" == "" ]; then
            echo "-T <site> or -m <map> should be specified"
            exit
	fi
	if [ $gazebo -eq 1 ] && [ "$world" == "" ]; then
            echo "-T <site> or -w <world> should be specified"
            exit
	fi
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
echo "Use gnss fix  : $use_gnss"


### For GAZEBO simulation, run wireless simulator with gazebo
### For physical robots, run wireless scanner separately
if [ $gazebo -eq 1 ]; then
  if [ "$wireless_config" == "" ]; then
    echo "does not launch ble_rss_simulator (world_config was not found)."
  else
    echo "launch gazebo helpers (ble_rss_simulator, floor_transition)"
    wireless_config=$(realpath $wireless_config)
    com="$command ros2 launch mf_localization_gazebo gazebo_helper.launch.py \
                    beacons_topic:=$beacons_topic \
                    wifi_topic:=$wifi_topic \
                    wireless_config_file:=$wireless_config \
                    $commandpost"
    echo $com
    eval $com
    pids+=($!)
    echo "${pids[@]}"
  fi
else
  # launch ublox node
  echo "launch ublox node helpers"
  eval "$command ros2 launch mf_localization ublox-zed-f9p.launch.xml \
                    $commandpost"

  pids+=($!)
fi

### launch rviz
if [ $show_rviz -eq 1 ]; then
   echo "launch rviz"
   eval "$command ros2 launch mf_localization view_multi_floor.launch.xml \
         use_sim_time:=$gazebo $commandpost"
   pids+=($!)
fi

if [ $cart_mapping -eq 1 ]; then
    cmd="$command ros2 launch mf_localization_mapping realtime_cartographer_2d_VLP16.launch.py \
          run_cartographer:=${RUN_CARTOGRAPHER:-true} \
          record_wireless:=true \
          save_samples:=true \
          record_required:=true \
          record_camera:=false \
          use_xsens:=${USE_XSENS:-true} \
          use_arduino:=${USE_ARDUINO:-false} \
          use_velodyne:=${USE_VELODYNE:-true} \
          imu_topic:=/imu/data \
          robot:=$robot_desc \
          use_sim_time:=$gazebo \
          bag_filename:=${OUTPUT_PREFIX}_`date +%Y-%m-%d-%H-%M-%S` $commandpost"
    echo $cmd
    eval $cmd
    pids+=($!)
fi

if [ $localization -eq 0 ]; then
    while [ 1 -eq 1 ];
    do
	snore 1
    done
    exit
fi

### launch localization
if [ $navigation -eq 0 ]; then
    # run mf_localization only
    launch_file="multi_floor_2d_rss_localization.launch.py"
    echo "launch $launch_file"
    com="$command ros2 launch mf_localization $launch_file \
                    robot:=$robot_desc \
                    map_config_file:=$map \
                    with_odom_topic:=true \
                    beacons_topic:=$beacons_topic \
                    wifi_topic:=$wifi_topic \
                    points2_topic:=$points2_topic \
                    imu_topic:=$imu_topic \
                    odom_topic:=$odom_topic \
                    pressure_available:=$([[ $pressure_available -eq 1 ]] && echo 'true' || echo 'false') \
                    pressure_topic:=$pressure_topic \
                    use_gnss:=$([[ $use_gnss -eq 1 ]] && echo 'true' || echo 'false') \
                    gnss_fix_topic:=$gnss_fix_topic \
                    gnss_fix_velocity_topic:=$gnss_fix_velocity_topic \
                    publish_current_rate:=$publish_current_rate \
                    use_sim_time:=$([[ $gazebo -eq 1 ]] && echo 'true' || echo 'false') \
                    $commandpost"
    echo $com
    eval $com
    pids+=($!)
    echo "${pids[@]}"

    # (daisueks) this is not required with ROS2
    # launch multi-floor map server for visualization
    if [ $map_server -eq 1 ]; then
        echo "launch multi_floor_map_server.launch"
        eval "$command ros2 launch mf_localization multi_floor_map_server.launch.xml \
                        map_config_file:=$map \
                        $commandpost"
        pids+=($!)
    fi
else
    # run navigation (mf_localization + planning)
    echo "launch multicart_demo.launch"
    eval "$command ros2 launch cabot_mf_localization multicart_demo.launch.py \
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

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
