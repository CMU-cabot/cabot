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


function red {
    echo -en "\033[31m"  ## red
    echo $1
    echo -en "\033[0m"  ## reset color
}

function usage {
    echo "Please specify a valid mode"
    echo ""
    echo " Usage:"
    echo "  $ $0 [mapping|topic_checker|localization]"
    echo ""
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

if [ $# -lt 1 ]; then
    usage
    exit
fi

case $1 in
    build)
	source /opt/underlay_ws/install_isolated/setup.bash
	catkin_make
	;;

    mapping)

	if [ "${OUTPUT_PREFIX}" = "" ]; then
            while [ 1 -eq 1 ]
            do
		red "You need to specify OUTPUT_PREFIX environment variable"
		snore 1
            done
	    exit
	fi

	shift
	source devel/setup.bash
	roscd cabot_mf_localization/script
	echo ./cabot_mf_localization.sh $@
	exec ./cabot_mf_localization.sh $@
	;;
    
    topic_checker)
	source devel/setup.bash
	rosnode list
	test=$?
	while [ $test -eq 1 ]; do
            snore 1
            c=$((c+1))
            rosnode list
            test=$?
	done
	exec rosrun mf_localization_mapping topic_checker.py
	;;
    
    localization)
	shift
	source devel/setup.bash
	roscd cabot_mf_localization/script
	exec ./cabot_mf_localization.sh $@
	;;
    
    *)
	red "There is no \"$1\" mode"
	usage
	;;
esac

