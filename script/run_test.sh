#!/bin/bash

# Copyright (c) 2020  Carnegie Mellon University
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

PKGS="cabot_ui cabot_msgs cabot_ui"

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

build=build
devel=devel
catkin_make=catkin_make
catkin_make_option1=test
catkin_make_option2=

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

while [ ${PWD##*/} != "catkin_ws" ]; do
    cd ..
done
catkin_ws=`pwd`
cd $catkin_ws

debug=0
function usage {
    echo "Usage"
    echo ""
    echo "-h                 show this help"
    echo "-d                 debug mode"
    echo "-p <packages>      specify a list of packages to be tested"
    echo "-i                 isolated"
}

while getopts "hdp:i" arg; do
    case $arg in
	h)
	    usage
	    exit
	    ;;
	d)
	    debug=1
	    ;;
	p)
	    PKGS=$OPTARG
	    ;;
	i)
	    build=build_isolated
	    devel=devel_isolated
	    catkin_make="catkin_make_isolated --use-ninja"
	    catkin_make_option2="--catkin-make-args test"
	    ;;
    esac
done
shift $((OPTIND-1))

for pkg in $PKGS
do
    echo "remove previous test results - " $pkg
    find $build/$pkg -name "*.xml" | \
	while read -r line; do
	    echo $line
	    rm $line
	done
    find $build/cabot_ros/$pkg -name "*.xml" | \
	while read -r line; do
	    echo $line
	    rm $line
	done
done


rosnode list
if [ $? -eq 1 ]; then
    xterm -e roscore &
    snore 5
fi

if [ $debug -eq 1 ]; then
    $catkin_make > /dev/null
    if [ $? -ne 0 ]; then
	echo "Compile error"
	exit
    fi
    $catkin_make $catkin_make_option1 --pkg $PKGS $catkin_make_option2 run_tests
else
    $catkin_make > /dev/null
    if [ $? -ne 0 ]; then
	echo "Compile error - try with -d option"
	exit
    fi
    $catkin_make $catkin_make_option1 --pkg $PKGS $catkin_make_option2 > /dev/null
fi

source $devel/setup.bash

echo ""
echo "=================Test Results====================="

#for pkg in $PKGS
#do
#    echo $pkg
catkin_test_results $build | grep Summary
catkin_test_results $build | grep xml | cut -f1 -d: | \
    while read -r line; do
	find $build -name `basename $line` | while read -r line; do
	    xsltproc ~/temp/NUnitXml.xslt $line
	done
    done
    
#done
