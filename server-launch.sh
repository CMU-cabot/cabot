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

trap ctrl_c INT TERM

function ctrl_c() {
    blue "exit script is hooked"
    log_name="MapData-$(date +%Y-%m-%d-%H-%M-%S).geojson"
    $DATA_SH -e "$scriptdir/.tmp/$log_name"
    blue "data saved"

    cd "$scriptdir" || exit
    ENV_FILE=$data_dir/server.env docker compose -f docker-compose-server.yaml down
    exit
}

function err {
    red >&2 "[ERROR] " "$@"
}
function red {
    echo -en "\033[31m" ## red
    echo "$@"
    echo -en "\033[0m" ## reset color
}
function blue {
    echo -en "\033[36m" ## blue
    echo "$@"
    echo -en "\033[0m" ## reset color
}
function snore() {
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read -r ${1:+-t "$1"} -u "$_snore_fd" || :
}

function help() {
    echo "Usage:"
    echo "-h          show this help"
    echo "-d <dir>    data directory"
    echo "-f          ignore errors"
}

data_dir=
ignore_error=0

while getopts "hd:f" arg; do
    case $arg in
    h)
        help
        exit
        ;;
    d)
        data_dir=$(realpath "$OPTARG")
        ;;
    f)
        ignore_error=1
        ;;
    *) ;;
    esac
done
shift $((OPTIND - 1))

## private variables

scriptdir=$(dirname "$0")
cd "$scriptdir" || exit
scriptdir=$(pwd)
temp_dir=$scriptdir/.tmp
mkdir -p "$temp_dir"

if [[ -z $data_dir ]]; then
    err "You should specify server data directory"
    help
    exit 1
fi

## check data file
error=0
files="server.env MapData.geojson"
for file in $files; do
    if [[ ! -e $data_dir/$file ]]; then
        err "$data_dir/$file file does not exist"
        error=1
    fi
done
# launch docker compose
if [[ ! -e $data_dir/server.env ]]; then
    error=1
    err "$data_dir/server.env file does not exist"
fi

if [[ $error -eq 1 ]] && [[ $ignore_error -eq 0 ]]; then
    err "add -f option to ignore file errors"
    exit 2
fi

export CABOT_SERVER_DATA_MOUNT=$data_dir
if [[ -e $data_dir/server.env ]]; then
    ENV_FILE=$data_dir/server.env docker compose -f docker-compose-server.yaml up -d
    ENV_FILE=$data_dir/server.env docker compose --ansi never -f docker-compose-server.yaml logs -f &
else
    docker compose -f docker-compose-server.yaml up -d
    docker compose --ansi never -f docker-compose-server.yaml logs -f &
fi

while true; do
    snore 1
done
