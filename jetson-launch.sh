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
    IFS=' '
    for conf in $config; do
        IFS=':'
        items=($conf)
        ipaddress=${items[1]}
        if [ $verbose -eq 1 ]; then
            com="ssh -l $user $ipaddress \"cd cabot; docker-compose -f docker-compose-jetson.yaml down\""
        else
            com="ssh -l $user $ipaddress \"cd cabot; docker-compose -f docker-compose-jetson.yaml down\" > /dev/null 2>&1"
        fi
        if [ $verbose -eq 1 ]; then
            echo $com
        fi
        eval $com
    done

    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            kill -s 2 $pid
        else
            kill -s 2 $pid > /dev/null 2>&1
        fi
    done
    for pid in ${pids[@]}; do
        if [ $verbose -eq 1 ]; then
            while kill -0 $pid; do
                snore 1
            done
        else
            while kill -0 $pid > /dev/null 2>&1; do
                snore 1
            done
        fi
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
    echo "              tracking  T:<ip>"
    echo "              detection D:<ip>:<ns>"
    echo "-v            verbose"
    echo ""
    echo "$0 -u <user> -c \"T:192.168.1.50 D:192.168.1.51:rs1 \""
}
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

user=
config=
testmode=0
testopt=
simulator=0
command="bash -c \""
commandpost="\"&"
verbose=0

while getopts "hdtsu:c:v" arg; do
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
        v)
            verbose=1
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
    if [ $verbose -eq 1 ]; then
        com="$command roscore $commandpost"
    else
        com="$command roscore > /dev/null 2>&1 $commandpost"
    fi

    eval $com
    pids+=($!)
fi

for conf in $config; do
    OLDIFS=$IFS
    IFS=':'
    items=($conf)
    mode=${items[0]}
    ipaddress=${items[1]}
    name=${items[2]}
    IFS=$OLDIFS

    if [ $verbose -eq 1 ]; then
	echo $conf $mode $ipaddress $name
    fi

    if [ "$mode" == 'D' ]; then
        blue "launch detect @ $ipaddress with $name"

        if [ -z $name ]; then
            red "You need to specify camera namespace"
            exit
        fi
        camopt="-r -D -v 3"
        if [ $simulator -eq 1 ]; then
            camopt="-s -D -v 3"
        fi

        if [ $verbose -eq 1 ]; then
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
$testopt \
$camopt \
-N ${name} \
-F 15 \
-f ${name}_link \\\" $commandpost"
        else
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
$testopt \
$camopt \
-N ${name} \
-F 15 \
-f ${name}_link \\\" > /dev/null 2>&1 $commandpost"
        fi
    elif [ "$mode" == 'T' ]; then
        blue "launch tracking @ $ipaddress"

        if [ $verbose -eq 1 ]; then
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
-K \\\" $commandpost"
        else
            com="$command ssh -l $user $ipaddress \
\\\"cd cabot; \
docker-compose -f docker-compose-jetson.yaml run --rm people-jetson /launch.sh \
-K \\\" > /dev/null 2>&1 $commandpost"
        fi
    else
	red "Unknown mode: $mode"
	exit
    fi

    if [ $verbose -eq 1 ]; then
        echo $com
    fi
    eval $com

    pids+=($!)
done

while [ 1 -eq 1 ];
do
    snore 1
done
