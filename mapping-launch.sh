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

function help()
{
    echo "Usage:"
    echo " this program runs recording for mapping"
    echo ""
    echo "-h          show this help"
    echo "-c          run realtime cartographer"
    echo "-a          use arduino for IMU topic"
    echo "-x          use xsens for IMU topic"
    echo "-o          output prefix"
}

OUTPUT_PREFIX=${OUTPUT_PREFIX:=mapping}
RUN_CARTOGRAPHER=false
USE_ARDUINO=false
USE_XSENS=false
USE_VELODYNE=true

while getopts "hcaxo:" arg; do
    case $arg in
        h)
            help
            exit
            ;;
	c)
	    RUN_CARTOGRAPHER=true
	    ;;
	a)
	    USE_ARDUINO=true
	    ;;
	x)
	    USE_XSENS=true
	    ;;
	o)
	    OUTPUT_PREFIX=$OPTARG
	    ;;
    esac
done
shift $((OPTIND-1))

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

echo "OUTPUT_PREFIX=$OUTPUT_PREFIX"
echo "RUN_CARTOGRAPHER=$RUN_CARTOGRAPHER"
echo "USE_ARDUINO=$USE_ARDUINO"
echo "USE_XSENS=$USE_XSENS"
echo "USE_VELODYNE=$USE_VELODYNE"


cd $scriptdir
export OUTPUT_PREFIX=$OUTPUT_PREFIX
export RUN_CARTOGRAPHER=$RUN_CARTOGRAPHER
export USE_ARDUINO=$USE_ARDUINO
export USE_XSENS=$USE_XSENS
export USE_VELODYNE=$USE_VELODYNE
docker-compose -f docker-compose-mapping.yaml up
