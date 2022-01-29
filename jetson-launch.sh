#!/bin/bash

# Copyright (c) 2021  Carnegie Mellon University
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

trap ctrl_c INT QUIT TERM HUP

function ctrl_c() {
    echo "trapped the signal"
    IFS=' '
    for conf in $config; do
	IFS=':'
	items=($conf)
	ipaddress=${items[0]}
	com="ssh -l $user $ipaddress \"cd cabot; docker-compose -f docker-compose-jetson.yaml down\""
	echo $com
	eval $com
    done

    for pid in ${pids[@]}; do
	echo "killing $pid..."
	kill -s 2 $pid
    done
    exit
}
function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}
function blue {
    echo -en "\033[36m"  ## blue
    echo $1
    echo -en "\033[0m"  ## reset color
}
function help {
    echo "Usage"
    echo ""
    echo "-h            show this help"
    echo "-d            debug mode, show in xterm"
    echo "-t            test mode, launch roscore, publish transform"
    echo "-s            run on simulator"
    echo "-u <user>     user name"
    echo "-c <config>   configuration"
    echo "              tracking  <ip>:t"
    echo "              detection <ip>:d:<ns>"
    echo ""
    echo "$0 -u <user> -c \"192.168.1.50:t 192.168.1.51:d:rs1 \""
}
pids=()

user=
config=
testmode=0
testopt=
simulator=0
command="bash -c \""
commandpost="\"&"
while getopts "hdtsu:c:" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	d)
	    command="setsid xterm -fa 'Monospace' -fs 11 -e \""
	    commandpost=";read\"&"
	    ;;
	t)
	    testmode=1
	    testopt="-t"
	    ;;
	s)
	    simulator=1
	    ;;
	u)
	    user=$OPTARG
	    ;;
	c)
	    config=$OPTARG
	    ;;
    esac
done

error=0
if [ -z "$user" ]; then
    red "Please specify user to login"
    error=1
fi

if [ -z "$config" ]; then
    red "Config is not specified"
    error=1
fi

if [ $error -eq 1 ]; then
    help
    exit
fi

if [ $testmode -eq 1 ]; then
    com="$command roscore $commandpost"
    eval $com
    pids+=($!)
fi

for conf in $config; do
    IFS=':'
    items=($conf)
    ipaddress=${items[0]}
    mode=${items[1]}
    name=${items[2]}

    if [ $mode == 'd' ]; then
	if [ -z $name ]; then
	    red "You need to specify camera namespace"
	    exit
	fi
	camopt="-r -D -v 3"
	if [ $simulator -eq 1 ]; then
	    camopt="-s -D -v 3"
	fi
	com="$command ssh -l $user $ipaddress \
	      	      \\\"cd cabot; \
                          docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
                                 $testopt \
				 $camopt \
                                 -N ${name} \
                                 -F 15 \
                                 -f ${name}_link \\\" $commandpost"
    elif [ $mode == 't' ]; then
	com="$command ssh -l $user $ipaddress \
	      	      \\\"cd cabot; \
                          docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
                                 -K \\\" $commandpost"
    fi
    echo $com
    eval $com

    pids+=($!)
done

while [ 1 -eq 1 ];
do
    snore 1
done
