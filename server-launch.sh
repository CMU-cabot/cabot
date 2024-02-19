#!/bin/bash

# Copyright (c) 2022  Carnegie Mellon University
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

trap ctrl_c INT TERM KILL

function ctrl_c() {
    blue "exit script is hooked"
    log_name="MapData-$(date +%Y-%m-%d-%H-%M-%S).geojson"
    $DATA_SH -e $scriptdir/.tmp/$log_name
    blue "data saved"

    cd $scriptdir
    ENV_FILE=$data_dir/server.env docker compose -f docker-compose-server.yaml down
    exit
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

function help()
{
    echo "Usage:"
    echo "-h           show this help"
    echo "-d <dir>     data directory"
    echo "-p <package> cabot site package name"
    echo "-f           ignore errors"
    echo "-v           verbose"
    echo "-c           clean the map_server before launch if the server is for different map"
    echo "-C           forcely clean the map_server"
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

data_dir=
ignore_error=0
verbose=0
clean_server=0

while getopts "hd:p:fvcC" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
            data_dir=$(realpath $OPTARG)
            ;;
	p)
	    cabot_site_dir=$(find $scriptdir/cabot_sites -name $OPTARG | head -1)
	    data_dir=${cabot_site_dir}/server_data
	    ;;
        f)
            ignore_error=1
            ;;
        v)
            verbose=1
            ;;
        c)
            clean_server=1
            ;;
        C)
            clean_server=2
            ;;
    esac
done
shift $((OPTIND-1))

## private variables
pids=()

temp_dir=$scriptdir/.tmp
mkdir -p $temp_dir


# forcely clean and extit
if [[ $clean_server -eq 2 ]]; then
    blue "Clean servers"
    for service in "map_server" "map_data" "mongodb"; do
        if [[ ! -z $(docker ps -f "name=$server" -q -a) ]]; then
	    docker ps -f "name=$server" -q -a | xargs docker stop
	    docker ps -f "name=$server" -q -a | xargs docker container rm
        fi
    done
    exit 0
fi


function check_server() {
    server=http://localhost:9090/map

    # check if the server data is same with the specified data
    curl $server/content-md5 --fail > ${temp_dir}/content-md5 2> /dev/null
    if [[ $? -ne 0 ]]; then
	blue "There is no server or servers may be launched by the old script."
	blue "Servers will be cleaned"
	return 1
    else
	cd $data_dir
	pwd
	md5sum=$(find . -type f ! -name 'content-md5' ! -name 'attachments.zip' -exec md5sum {} + | LC_COLLATE=C sort -k 2 | md5sum)
	cd $scriptdir
	blue "md5sum - $md5sum"
	blue "server - $(cat ${temp_dir}/content-md5)"
	if [[ $(cat ${temp_dir}/content-md5) == $md5sum ]]; then  ## match, so no clean
	    blue "md5 matched, do not relaunch server"
	    return 0
	fi
    fi
    return 2
}

if [ ! -e $data_dir ]; then
    err "You should specify correct server data directory or cabot site package name"
    help
    exit 1
fi

if [[ $clean_server -eq 1 ]]; then
    if [ -z $data_dir ]; then
	err "You should specify correct server data directory or cabot site package name"
	help
	exit 1
    fi

    if check_server; then
	exit 0
    else
	blue "Clean servers"
	for service in "map_server" "map_data" "mongodb"; do
            if [[ ! -z $(docker ps -f "name=$service" -q -a) ]]; then
		docker ps -f "name=$service" -q -a | xargs docker stop
		docker ps -f "name=$service" -q -a | xargs docker container rm
            fi
	done
    fi
else
    flag=0
    if check_server; then
	exit 0
    fi

    for service in "map_server" "map_data" "mongodb"; do
        if [[ $(docker ps -f "name=$service" -q | wc -l) -ne 0 ]]; then
            err "There is $service server running"
            flag=1
        fi
    done
    if [[ $flag -eq 1 ]]; then
        red "Please stop the servers with '-C' or '-c' option to clean before launch"
        exit 1
    fi
fi

## check data file
error=0
files="server.env MapData.geojson"
for file in $files; do
    if [ ! -e $data_dir/$file ]; then
        err "$data_dir/$file file does not exist";
        error=1;
    fi
done
# launch docker compose
if [ ! -e $data_dir/server.env ]; then
    error=1
    err "$data_dir/server.env file does not exist"
fi

if [ $error -eq 1 ] && [ $ignore_error -eq 0 ]; then
   err "add -f option to ignore file errors"
   exit 2
fi

export CABOT_SERVER_DATA_MOUNT=$data_dir
if [ -e $data_dir/server.env ]; then
    if [[ $verbose -eq 1 ]]; then
        ENV_FILE=$data_dir/server.env docker compose -f docker-compose-server.yaml up -d
        ENV_FILE=$data_dir/server.env docker compose --ansi never -f docker-compose-server.yaml logs -f
    else
        ENV_FILE=$data_dir/server.env docker compose -f docker-compose-server.yaml up -d
    fi
else
    if [[ $verbose -eq 1 ]]; then
        docker compose -f docker-compose-server.yaml up -d
        docker compose --ansi never -f docker-compose-server.yaml logs -f
    else
        docker compose -f docker-compose-server.yaml up -d
    fi
fi
