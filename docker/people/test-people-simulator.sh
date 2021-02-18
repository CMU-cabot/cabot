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

# check if necessary command exists
if ! type "realpath" > /dev/null; then
  echo "realpath command is not installed. Please install by apt-get install realpath"
  exit
fi

## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    ## killing simulator node
    for pid in ${pids[@]}; do
    	echo "killing $pid..."
    	kill -s 15 $pid
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

function is_running() {
    for pid in ${pids[@]}; do
      if ps -p $pid > /dev/null
      then
        return 0
      fi
    done
    return 1
}

## private variables
pids=()
record_pid=

### default variables
test_id=
site=cabot_site_coredo_3d
master_ip=
ros_ip=
output_bag=
max_sim_time=100

queue=""

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running setup-people-simulator.py in another terminal"
    echo "ex)"
    echo $0 "-s <site package> -t <test ID> -m <Master IP> -r <ROS IP> -M <max sim time>"
    echo ""
    echo "-h                       show this help"
    echo "-s <site package>        packge name for the robot site"
    echo "-t <test ID>             people simulation test ID"
    echo "-m <Master IP>           Master PC IP address"
    echo "-r <ROS IP>              ROS PC IP address"
    echo "-o <output bag>          output ROS bag file"
    echo "-M <max sim time>        maximum simulation time"
    echo "-q                       queue simulation mode"
    exit
}

while getopts "hs:t:m:r:o:M:q" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	s)
	    site=$OPTARG
	    ;;
	t)
	    test_id=$OPTARG
	    ;;
	m)
	    master_ip=$OPTARG
	    ;;
	r)
	    ros_ip=$OPTARG
	    ;;
	o)
	    output_bag=`realpath $OPTARG`
	    ;;
	M)
	    max_sim_time=$OPTARG
	    ;;
	q)
      queue="-q"
	    ;;
  esac
done
shift $((OPTIND-1))


## debug output
echo "Site       : $site"
echo "Test ID    : $test_id"
echo "Master IP  : $master_ip"
echo "ROS IP     : $ros_ip"
echo "Output bag : $output_bag"
echo "Maximum simulation time : $max_sim_time"
echo "Queue mode : $queue"

if [ "$output_bag" == "" ]; then
  echo "Error. Output ROS bag file path is not valid."
  exit
fi
if [ -f "$output_bag" ]; then
  echo "Error. Output ROS bag file already exist."
  exit
fi

# record ROS bag file
if [ "$output_bag" != "" ]; then
  echo "record rosbag file as $output_bag"
  if [ $queue == "-q" ]; then
    eval "rosbag record /queue_people_py/queue \
          /people /cmd_vel /current_floor /current_frame /navigate_to_pose/status /navigate_to_pose/goal \
          -O $output_bag __name:=people_sim_bag &"
    record_pid=$!
  else
    eval "rosbag record /people_simulator/simulator_status /people_simulator/robot_pose /people_simulator/people \
          /people /cmd_vel /current_floor /current_frame /navigate_to_pose/status /navigate_to_pose/goal \
	  /cabot/cmd_vel /cabot/cmd_vel_limited /cabot/lidar_speed /cabot/people_speed /cabot/tf_speed /cabot/user_speed \
          -O $output_bag __name:=people_sim_bag &"
    record_pid=$!
  fi
fi

# run people simulator
if [ "$site" == "" ]; then
  echo "-s <site> should be specified for people simulation"
  exit
fi
if [ "$test_id" == "" ]; then
  echo "-t <test ID> should be specified for people simulation"
  exit
fi
if [ "$master_ip" == "" ]; then
  echo "-m <Master IP> should be specified for people simulation"
  exit
fi
if [ "$ros_ip" == "" ]; then
  echo "-r <ROS IP> should be specified for people simulation"
  exit
fi
echo "Run 'env MASTER_IP=$master_ip ROS_IP=$ros_ip docker-compose exec people /peoplesim.sh -T $site -s -p $test_id -M $max_sim_time $queue'"
eval "env MASTER_IP=$master_ip ROS_IP=$ros_ip docker-compose exec people /peoplesim.sh -T $site -s -p $test_id -M $max_sim_time $queue"
pids+=($!)
if [ "$record_pid" != "" ]; then
  pids=("${pids[@]/$record_pid}")
fi

## wait until it is terminated automatically or by the user
while is_running
do
  snore 1
done

# stop rosbag record
if [ "$record_pid" != "" ]; then
  echo "stop rosbag record..."
  rosnode kill /people_sim_bag
fi
