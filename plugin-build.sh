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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

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
    echo "  $0 -m <cabot-model> [-c <custom.yaml>]"
    echo ""
    echo "    -h                show this help"
    echo "    -m <cabot-model>  specify cabot model or use .env to set CABOT_MODEL"
    echo "    -c <custom.yaml>  specify custom.yaml to override default plugins.yaml"
}

# load .env file first
if [[ -e .env ]]; then
    source .env
fi

custom_yaml=

while getopts "hm:c:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
        m)
            CABOT_MODEL=$OPTARG
            ;;
        c)
            custom_yaml=$OPTARG
            ;;
    esac
done

if [[ -z ${CABOT_MODEL} ]]; then
    help
    exit
fi

blue "CABOT_MODEL=${CABOT_MODEL}"
if [[ -n ${custom_yaml} ]]; then
    blue "CUSTOM_YAML=${custom_yaml}"
fi

# update .env file
if [[ ! -e .env ]]; then
    echo "CABOT_MODEL=${CABOT_MODEL}" > .env
else
    if grep -q '^CABOT_MODEL=' .env; then
        sed -i "s/^CABOT_MODEL=.*/CABOT_MODEL=${CABOT_MODEL}/" .env
    else
        echo "CABOT_MODEL=${CABOT_MODEL}" >> .env
    fi
fi

if [[ -n ${custom_yaml} ]]; then
    ./tools/plugin-build.py -m ${CABOT_MODEL} -c ${custom_yaml}
else
    ./tools/plugin-build.py -m ${CABOT_MODEL}
fi
