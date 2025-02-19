#!/bin/bash

# Copyright (c) 2024  Carnegie Mellon University
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
start=`date +%s.%N`
trap ctrl_c INT QUIT TERM

terminating=0
launched=0

function ctrl_c() {
    red "catch the signal"
    user=$1
    terminating=1
    cd $scriptdir
    if [[ ! -z $dccom ]]; then
        while [[ $launched -lt 5 ]]; do
            snore 1
            launched=$((launched+1))
        done

        red "$dccom down"
        if [ $verbose -eq 1 ]; then
            $dccom down
        else
            $dccom down > /dev/null 2>&1
        fi
    fi
    exit $user
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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

verbose=0

while getopts "hv" arg; do
    case $arg in
        h)
            echo "Usage:"
            echo "  $0 [-v]"
            echo ""
            echo "    -h  show this help"
            echo "    -v  verbose mode"
            exit
            ;;
        v)
            verbose=1
            ;;
    esac
done

log_name=cabot_plugins_`date +%Y-%m-%d-%H-%M-%S`
log_dir=$scriptdir/log
mkdir -p $log_dir
log_file=$log_dir/$log_name

dcfile="docker-compose-plugins.yaml"
dccom="docker compose -f $dcfile"

if [ $verbose -eq 0 ]; then
    com2="bash -c \"setsid $dccom --ansi never up --no-build\" > $log_file &"
else
    com2="bash -c \"setsid $dccom up --no-build\" | tee $log_file &"
fi
if [ $verbose -eq 1 ]; then
    blue "$com2"
fi

if [[ $terminating -eq 1 ]]; then
    exit
fi

eval $com2
dcpid=($!)
while [[ $launched -lt 5 ]]; do
    snore 1
    launched=$((launched+1))
done

while [ 1 -eq 1 ];
do
    # check if any of container got Exit status
    if [[ $terminating -eq 0 ]] && [[ `$dccom ps | grep Exit | wc -l` -gt 0 ]]; then
        red "docker compose may have some issues. Check errors in the log or run with '-v' option."
        ctrl_c 1
        exit
    fi
    snore 1
done
