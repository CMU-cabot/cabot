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
output_log_dir=
visualize=0
commandpost=''

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running setup-people-simulator.py in another terminal"
    echo "ex)"
    echo $0 "-s <site package> -t <test IDs> -o <output directory>"
    echo ""
    echo "-h                           show this help"
    echo "-s <site package>            packge name for the robot site"
    echo "-t <test IDs>                people simulation test ID list separated by comma"
    echo "-o <output log directory>    output directory which has ROS bag files"
    echo "-v                           turn on visualization"
    exit
}

while getopts "hs:t:o:v" arg; do
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
	o)
      output_log_dir=$OPTARG
	    ;;
	v)
	    visualize=1
      commandpost="--visualize"
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
echo "Output log directory : $output_log_dir"

if [ "$output_log_dir" == "" ]; then
  echo "Error. Output log directory path is not valid."
  exit
fi

# clear evaluation results
echo "Run 'rm $output_log_dir/result.csv'"
rm $output_log_dir/result.csv
echo "Run 'rm $output_log_dir/*.png'"
rm $output_log_dir/*.png

for test_id in ${test_id_array[@]};
do
  echo "Run 'python $scriptdir/eval-people-simulator.py -r $output_log_dir/$test_id.bag -s $site -t $test_id -o $output_log_dir/result.csv $commandpost'"
  eval "python $scriptdir/eval-people-simulator.py -r $output_log_dir/$test_id.bag -s $site -t $test_id -o $output_log_dir/result.csv $commandpost"
  pids+=($!)
  
  ## wait until evaluation finish
  while is_running
  do
    snore 1
  done
done
