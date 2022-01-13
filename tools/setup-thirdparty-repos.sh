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

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    show this help"
    echo "-c                    clean (rm -rf) repositories"
}

clean=0

while getopts "hc" arg; do
    case $arg in
	h)
	    help
	    exit
	    ;;
	c)
	    clean=1
	    ;;
    esac
done


cd $scriptdir/../
if [ $clean -eq 1 ]; then
    repos=`python3 -c "import yaml;print('\t'.join(list(map(lambda x: x, yaml.safe_load(open('thirdparty.repos'))['repositories']))))"`
    for repo in $repos; do
	echo "rm -rf $repo"
	rm -rf $repo
    done
else
    vcs import < thirdparty.repos
fi

cd $scriptdir/../docker
if [ $clean -eq 1 ]; then
    repos=`python3 -c "import yaml;print('\t'.join(list(map(lambda x: x, yaml.safe_load(open('thirdparty.repos'))['repositories']))))"`
    for repo in $repos; do
	echo "rm -rf $repo"
	rm -rf $repo
    done
else
    vcs import < thirdparty.repos
fi
