#!/bin/bash


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

pid=$(cat /tmp/cabot.pid)

echo $pid
kill -2 $pid


while [[ 1 -eq 1 ]]; do
    while kill -0 $dcpid 2> /dev/null; do
        snore 1
    done
    exit 0
done
