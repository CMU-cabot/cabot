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

pwd=`pwd`
scriptdir=`dirname $0`

### default variables
test_id_list=101,102,103,104,105,106,107,108,109,110,111,112,201,202,203,204,205,206,207,208,209,210,211,212,301,302,303,401,402
test_id_array=${test_id_list//,/ }
site=cabot_site_coredo_3d
master_ip=
ros_ip=
output_dir=
max_sim_time=100

queue=""

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running setup-people-simulator.py in another terminal"
    echo "ex)"
    echo $0 "-s <site package> -t <test IDs> -m <Master IP> -r <ROS IP> -o <output directory> -M <max sim time>"
    echo ""
    echo "-h                       show this help"
    echo "-s <site package>        packge name for the robot site"
    echo "-t <test IDs>            people simulation test ID list separated by comma"
    echo "-m <Master IP>           Master PC IP address"
    echo "-r <ROS IP>              ROS PC IP address"
    echo "-o <output directory>    output directory"
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
      test_id_list=$OPTARG
      test_id_array=${test_id_list//,/ }      
	    ;;
	m)
	    master_ip=$OPTARG
	    ;;
	r)
	    ros_ip=$OPTARG
	    ;;
	o)
	    output_dir=`realpath $OPTARG`
      timestamp=`date +%Y-%m-%d-%H-%M-%S`
      output_log_dir="$output_dir/$timestamp"
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
echo "Site             : $site"
echo "List of test IDs : "
for test_id in ${test_id_array[@]};
do
  echo $test_id;
done
echo "Master IP        : $master_ip"
echo "ROS IP           : $ros_ip"
echo "Output directory : $output_dir"
echo "Output log directory : $output_log_dir"
echo "Maximum simulation time : $max_sim_time"
echo "Queue mode : $queue"

if [ "$output_dir" == "" ]; then
  echo "Error. Output directory path is not valid."
  exit
fi

mkdir -p $output_log_dir

for test_id in ${test_id_array[@]};
do
  echo "Run 'bash $scriptdir/test-people-simulator.sh -m $master_ip -r $ros_ip -s $site -t $test_id -o $output_log_dir/$test_id.bag -M $max_sim_time $queue'"
  eval "bash $scriptdir/test-people-simulator.sh -m $master_ip -r $ros_ip -s $site -t $test_id -o $output_log_dir/$test_id.bag -M $max_sim_time $queue"
  pids+=($!)
  
  ## wait until simulation finish
  while is_running
  do
    snore 1
  done

  echo "Run 'python $scriptdir/eval-people-simulator.py -r $output_log_dir/$test_id.bag -s $site -t $test_id -o $output_log_dir/result.csv'"
  eval "python $scriptdir/eval-people-simulator.py -r $output_log_dir/$test_id.bag -s $site -t $test_id -o $output_log_dir/result.csv"
  pids+=($!)
  
  ## wait until evaluation finish
  while is_running
  do
    snore 1
  done
done
