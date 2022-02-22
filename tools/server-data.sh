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
    echo "Usage: "
    echo " $0 -i <file>: import data from file"
    echo " $0 -e <file>: export data to file"
    echo " $0 -d       : delete all data on the server"
    echo ""
    echo "-h          show this help"
    echo "-f          force to overwrite file if file exists"
}

action=
file_path=
force=0

while getopts "hi:e:df" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        i)
	    file_path=$(realpath $OPTARG)
	    action=import
	    ;;
	e)
	    file_path=$(realpath $OPTARG)
	    action=export
	    ;;
	d)
	    action=delete-all
            ;;
	f)
	    force=1
	    ;;
    esac
done
shift $((OPTIND-1))

if [ -z $action ]; then
    help
    exit 2
fi

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
temp_dir=$scriptdir/../.tmp
mkdir -p $temp_dir

HOST=http://localhost:9090/map
API_KEY=local-server-editor-api-key

cd $temp_dir

if [ "$action" = "import" ]; then
    blue "importing $file_path"
    if [ ! -e $file_path ]; then
	err "import: $file_path does not exists"
	exit 3
    fi
    cat $file_path | jq .features > insert.json
    curl -X POST \
	 -H "Content-Type: application/x-www-form-urlencoded; charset=utf-8" \
	 --data-urlencode  "editor_api_key=$API_KEY" \
	 --data-urlencode  "user=script" \
	 --data-urlencode  "lang=en" \
	 --data-urlencode  "action=editdata" \
	 --data-urlencode  "remove=[]" \
	 --data-urlencode  "update=[]" \
	 --data-urlencode  insert@insert.json \
	 "$HOST/api/editor" > /dev/null 2>&1
elif [ "$action" = "export" ]; then
    blue "exporting to $file_path"
    if [ -e $file_path ] && [ $force -eq 0 ]; then
	err "export: $file_path already exists, use -f option to overwrite"
	exit 4
    fi
    curl "$HOST/api/config" > config.json 2> /dev/null
    LAT=$(jq .INITIAL_LOCATION.lat config.json)
    LNG=$(jq .INITIAL_LOCATION.lng config.json)
    options="user=script&dist=1000&lat=$LAT&lng=$LNG&lang=en&cache=false&editor_api_key=$API_KEY"
    curl "$HOST/routesearch?action=start&$options" > /dev/null 2>&1
    curl "$HOST/routesearch?action=nodemap&$options" 2> /dev/null | jq '[ .[] ]' > nodemap.json
    curl "$HOST/routesearch?action=features&$options" > features.json 2> /dev/null
    jq -s '. | add' nodemap.json features.json > all.json
    jq --tab '{"type": "FeatureCollection", "features": .}' all.json > $file_path
elif [ "$action" = "delete-all" ]; then
    blue "deleting all data"
    curl "$HOST/api/config" > config.json 2> /dev/null
    LAT=$(jq .INITIAL_LOCATION.lat config.json)
    LNG=$(jq .INITIAL_LOCATION.lng config.json)
    options="user=script&dist=1000&lat=$LAT&lng=$LNG&lang=en&cache=false&editor_api_key=$API_KEY"
    curl "$HOST/routesearch?action=start&$options" > /dev/null 2>&1
    curl "$HOST/routesearch?action=nodemap&$options" 2> /dev/null | jq '[ .[] ]' > nodemap.json
    curl "$HOST/routesearch?action=features&$options" > features.json 2> /dev/null
    jq -s '. | add' nodemap.json features.json > remove.json
    curl -X POST \
	 --data-urlencode  "editor_api_key=$API_KEY" \
	 --data-urlencode  "user=script" \
	 --data-urlencode  "lang=en" \
	 --data-urlencode  "action=editdata" \
	 --data-urlencode  remove@remove.json \
	 --data-urlencode  "update=[]" \
	 --data-urlencode  "insert=[]" \
	 "$HOST/api/editor" > /dev/null 2>&1
fi
