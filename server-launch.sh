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
    ENV_FILE=$data_dir/server.env docker-compose -f docker-compose-server.yaml down
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
    echo "-h          show this help"
    echo "-d <dir>    data directory"
}

data_dir=

while getopts "hd:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        d)
	    data_dir=$(realpath $OPTARG)
            ;;
    esac
done
shift $((OPTIND-1))

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
temp_dir=$scriptdir/.tmp
mkdir -p $temp_dir

DATA_SH=$scriptdir/tools/server-data.sh

if [ -z $data_dir ]; then
    err "You should specify server data directory"
    help
    exit 1
fi

## check data file
error=0
files="server.env attachments.zip MapData.geojson"
for file in $files; do
    if [ ! -e $data_dir/$file ]; then
	err "$data_dir/$file file does not exist";
	error=1;
    fi
done
if [ $error -eq 1 ]; then
   exit 2
fi

# launch docker-compose
ENV_FILE=$data_dir/server.env docker-compose -f docker-compose-server.yaml up -d
ENV_FILE=$data_dir/server.env docker-compose --ansi never -f docker-compose-server.yaml logs -f &


HOST=http://localhost:9090/map
admin=hulopadmin
pass=please+change+password
editor=editor

count=0
echo "waiting server is up"
while [ "$(curl -I $HOST/login.jsp 2>/dev/null | head -n 1 | cut -d' ' -f2)" != "200" ];
do
    snore 1
    UPLINE=$(tput cuu1)
    ERASELINE=$(tput el)
    echo -n "$UPLINE$ERASELINE"
    echo "waiting server is up ($count)"
    count=$((count+1))
done

pushd $temp_dir > /dev/null

blue "adding editor user"
curl -b admin-cookie.txt -c admin-cookie.txt $HOST/admin.jsp > /dev/null 2>&1
curl -b admin-cookie.txt -c admin-cookie.txt -d "redirect_url=admin.jsp&user=${admin}&password=${pass}" $HOST/login.jsp > /dev/null 2>&1
curl -b admin-cookie.txt -d "user=$editor&password=$editor&password2=$edito&role=editor" "$HOST/api/user?action=add-user" > /dev/null 2>&1

blue "importing attachments.zip"
if [ -e $data_dir/attachments.zip ]; then
    curl -b admin-cookie.txt -c admin-cookie.txt $HOST/admin.jsp > /dev/null 2>&1
    curl -b admin-cookie.txt -c admin-cookie.txt -d "redirect_url=admin.jsp&user=${admin}&password=${pass}" $HOST/login.jsp > /dev/null 2>&1
    curl -b admin-cookie.txt -F file=@$data_dir/attachments.zip "$HOST/api/admin?action=import&type=attachment.zip" > /dev/null 2>&1
fi

$DATA_SH -i $data_dir/MapData.geojson import

while [ 1 -eq 1 ];
do
    snore 1
done
