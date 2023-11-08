#!/bin/bash

# Copyright (c) 2023  Carnegie Mellon University
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

function blue {
    echo -en "\033[36m"  ## blue
    echo $@
    echo -en "\033[0m"  ## reset color
}

function help {
    echo "Usage: $0 <option>"
    echo ""
    echo "-h                    show this help"
    echo "-c                    clean (rm -rf) dependency repositories"
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


if [[ $clean -eq 1 ]]; then
    find * -name ".git" | while read -r line; do
	echo "rm -rf $(dirname $line)"
	rm -rf $(dirname $line)
    done
    exit
fi

declare -A visited

while true; do
    files=$(find . -name "dependency.repos")

    flag=0
    for line in ${files[@]}; do
	if [[ -z ${visited[$line]} ]]; then
	    flag=1
	    visited[$line]=1
	    
	    pushd $(dirname $line)
	    blue "vcs import < $(basename $line)"
	    vcs import < $(basename $line)
	    popd
	fi
    done
    if [[ $flag -eq 0 ]]; then
	break
    fi
done
