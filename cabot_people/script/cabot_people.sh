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


## termination hook
trap ctrl_c INT QUIT TERM

function ctrl_c() {
    ## killing all nodes
    #echo "killing all ros nodes..."
    #rosnode kill -a

    for pid in ${pids[@]}; do
        echo "killing $pid..."
        kill -s 2 $pid
    done
    rlc=0
    while [ `ps -A | grep  roslaunch | wc -l` -ne 0 ];
    do
        snore 1
        echo -ne "waiting nodes are completely terminated ($rlc)"\\r
        rlc=$((rlc+1))
    done
    echo \\n
    exit
} 

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

## private variables
pids=()

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`

### default variables

## debug
minimum=0
debug=0
command=''
commandpost='&'

: ${CABOT_GAZEBO:=0}
: ${CABOT_SITE:=}
: ${CABOT_USE_REALSENSE:=0}
: ${CABOT_SHOW_PEOPLE_RVIZ:=0}
: ${CABOT_REALSENSE_SERIAL:=}
: ${CABOT_CAMERA_NAME:=camera}
: ${CABOT_CAMERA_RGB_FPS:=15.0}  # should be double
: ${CABOT_CAMERA_DEPTH_FPS:=15.0}
: ${CABOT_CAMERA_RESOLUTION:=1280}
: ${CABOT_DETECT_VERSION:=3}

gazebo=$CABOT_GAZEBO
site=$CABOT_SITE
show_rviz=$CABOT_SHOW_PEOPLE_RVIZ
realsense_camera=$CABOT_USE_REALSENSE
serial_no=$CABOT_REALSENSE_SERIAL

namespace=$CABOT_CAMERA_NAME
camera_link_frame="${CABOT_CAMERA_NAME}_link"

rgb_fps=$CABOT_CAMERA_RGB_FPS
depth_fps=$CABOT_CAMERA_DEPTH_FPS
resolution=$CABOT_CAMERA_RESOLUTION

opencv_dnn_ver=$CABOT_DETECT_VERSION

queue_detector=0
check_required=0
publish_tf=0
publish_sim_people=0
wait_roscore=0
roll=0
tracking=0
detection=0
obstacle=0
noreset=0

### usage print function
function usage {
    echo "Usage"
    echo "    run this script after running cabot.sh in another terminal"
    echo "ex)"
    echo $0 "-T <site_package>"
    echo ""
    echo "-h                       show this help"
    echo "-d                       debug"
    echo "-V                       show rviz"
    echo "-m <map file>            specify a map file"
    echo "-n <anchor file>         specify a anchor file, use map file if not specified"
    echo "-w <world file>          specify a world file"
    echo "-s                       specify its on simulation (gazebo)"
    echo "-r                       launch realsense camera"
    echo "-q                       use queue detector"
    echo "-T <site package>        packge name for the robot site (only for queue)"
    echo "-p                       publish simulation people instead of detected people from camera"
    echo "-K                       use people tracker"
    echo "-D                       use people detector"
    echo "-C                       check required before launch"
    echo "-W                       wait roscore"
    echo "-t <roll>                publish map camera_link tf"
    echo "-v [1-3]                 use specified opencv dnn implementation"
    echo "   1: python-opencv, 2: cpp-opencv-node, 3: cpp-opencv-nodelet"
    echo "-N <name space>          namespace for tracking"
    echo "-f <camera_link_frame>   specify camera link frame"
    echo "-F <fps>                 specify camera RGB fps"
    echo "-P <fps>                 specify camera depth fps"
    echo "-S <camera serial>       specify serial number of realsense camera"
    echo "-R 1280/848/640          specify camera resolution"
    echo "-O                       obstacle detection/tracking"
    echo "-a                       no resetrs each"
    exit
}

while getopts "hdm:n:w:srqVT:Ct:pWv:N:f:KDF:P:S:R:Oa" arg; do
    case $arg in
    h)
        usage
        exit
        ;;
    d)
        debug=1
        command="setsid xterm -e '"
        commandpost=";read'&"
        ;;
    m)
        map=$OPTARG
        ;;
    n)
        anchor=$OPTARG
        ;;
    w)
        world=$OPTARG
        ;;
    s)
        gazebo=1
        ;;
    r)
        realsense_camera=1
        ;;
    q)
        queue_detector=1
        ;;
    V)
        show_rviz=1
        ;;
    T)
        site=$OPTARG
        ;;
    C)
        check_required=1
        ;;
    t)
        publish_tf=1
	    roll=$OPTARG
        ;;
    p)
        publish_sim_people=1
        ;;
    W)
        wait_roscore=1
        ;;
    v)
        opencv_dnn_ver=$OPTARG
        ;;
    N)
        namespace=$OPTARG
        ;;
    f)
        camera_link_frame=$OPTARG
        ;;
    K)
        tracking=1
        ;;
    D)
        detection=1
        ;;
    F)
        rgb_fps=$OPTARG
        ;;
    P)
        depth_fps=$OPTARG
        ;;
    S)
        serial_no=$OPTARG
        ;;
    R)
        resolution=$OPTARG
        ;;
    O)
        obstacle=1
        ;;
    a)
	noreset=1
	;;
    esac
done
shift $((OPTIND-1))

width=$resolution
if [ $width -eq 1280 ]; then
    height=720
elif [ $width -eq 848 ]; then
    height=480
elif [ $width -eq 640 ]; then
    height=360
else
    red "resolution should be one of 1280, 848, or 640"
    exit
fi

if [ $check_required -eq 1 ]; then
    flag=1
    while [ $flag -eq 1 ];
    do
        flag=0

        if [ $gazebo -eq 1 ]; then
            # Check Gazebo
            if [ `rostopic list | grep gazebo | wc -l` -eq 0 ]; then
                echo "Gazebo is not working"
                flag=1
            fi
        else
            # Check RealSense
            if [ `rs-fw-update -l | grep D435 | wc -l` -eq 0 ]; then
                echo "Realsense is not working"
                flag=1
            fi
        fi
        
        # Check weight
        if [ `find $scriptdir/../../ -name yolov4.weights | wc -l` -eq 0 ]; then
            echo "yolov4.weights is not found"
            flag=1
        fi
        
        snore 2
    done
fi

# load site package
if [ $queue_detector -eq 1 ]; then
   if [ "$site" != "" ]; then
       sitedir=`ros2 pkg prefix $site`/share/$site
       source $sitedir/config/config.sh
       if [ "$map" == "" ] && [ "$world" == "" ]; then
           echo "Please check config/config.sh in site package ($sitedir) to set map and world"
           exit
       fi
   else
       if [ "$map" == "" ]; then
           echo "-T <site> or -m <map> should be specified"
           exit
       fi
       if [ $gazebo -eq 1 ] && [ "$world" == "" ]; then
           echo "-T <site> or -w <world> should be specified"
           exit
       fi
   fi
fi


## debug output
echo "Use Realsense : $realsense_camera"
echo "Debug         : $debug ($command, $commandpost)"
echo "World         : $world"
echo "Map           : $map"
echo "Anchor        : $anchor"
echo "Simulation    : $gazebo"
echo "DNN impl      : $opencv_dnn_ver"
echo "Namespace     : $namespace"
echo "Camera frame  : $camera_link_frame"
echo "RGB FPS       : $rgb_fps"
echo "Depth FPS     : $depth_fps"
echo "Resolution    : $width x $height"
echo "Obstacle      : $obstacle"


if [ $publish_tf -eq 1 ]; then
    eval "$command ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 $roll map ${camera_link_frame} $commandpost"
    pids+=($!)
fi

### launch rviz2
if [ $show_rviz -eq 1 ]; then
    echo "launch rviz2"
    eval "$command rviz2 -d $scriptdir/cabot_people.rviz $commandpost"
    pids+=($!)
fi

### launch realsense camera
if [ $realsense_camera -eq 1 ]; then

    if [ $noreset -eq 0 ]; then
    	# reset RealSense port
    	sudo /resetrs.sh $serial_no
    fi

    option=""
    # work around to specify number string as string
    if [[ ! -z $serial_no ]]; then option="$option \"serial_no:='$serial_no'\""; fi
    launch_file="realsense2_camera rs_launch.py"
    echo "launch $launch_file"
    eval "$command ros2 launch $launch_file \
                   align_depth:=true \
                   depth_fps:=$depth_fps \
                   color_fps:=$rgb_fps \
                   depth_width:=$width \
                   color_width:=$width \
                   depth_height:=$height \
                   color_height:=$height \
		   $option \
                   camera_name:=${namespace} $commandpost"
    pids+=($!)
fi

opt_predict=''

if [ $detection -eq 1 ]; then
    ### launch people detect
    map_frame='map'
    depth_registered_topic=''
    if [ $gazebo -eq 1 ]; then
        depth_registered_topic='/${namespace}/depth/image_raw'
    fi
        
    if [ $opencv_dnn_ver -ge 2 ]; then
        use_nodelet=0

	# do not use nodelet if it is on gazebo
        if [ $gazebo -eq 0 ] && [ $opencv_dnn_ver -eq 3 ]; then
            use_nodelet=1
        fi
	# cpp
        eval "$command ros2 launch track_people_cpp detect_darknet_nodelet.launch.xml \
                       namespace:=$namespace \
                       map_frame:=$map_frame \
                       camera_link_frame:=$camera_link_frame \
                       use_nodelet:=$use_nodelet \
                       depth_registered_topic:=$depth_registered_topic \
                       $commandpost"
	pids+=($!)
    else
	# python
        launch_file="track_people_py detect_darknet_realsense.launch.xml"
        echo "launch $launch_file"
        eval "$command ros2 launch $launch_file \
                       namespace:=$namespace \
                       map_frame:=$map_frame \
                       camera_link_frame:=$camera_link_frame \
                       depth_registered_topic:=$depth_registered_topic \
                       $commandpost"
	pids+=($!)
    fi

fi

if [ $tracking -eq 1 ]; then
    ### launch people track
    launch_file="track_people_py track_sort_3d.launch.xml"
    echo "launch $launch_file"
    eval "$command ros2 launch $launch_file \
                $commandpost"
    pids+=($!)

    ### launch people predict
    opt_predict=''
    if [ $gazebo -eq 1 ] && [ $publish_sim_people -eq 1 ]; then
        opt_predict='publish_simulator_people:=true'
    fi
    launch_file="predict_people_py predict_kf.launch"
    echo "launch $launch_file"
    eval "$command ros2 launch $launch_file $opt_predict \
                   $commandpost"
    pids+=($!)
fi

### launch queue detect
if [ $queue_detector -eq 1 ]; then
    if [ "$sitedir" == "" ]; then
        echo "-T <site> should be specified for queue detect"
        exit
    fi
    if [ $gazebo -eq 1 ]; then
        queue_det_config_file=$sitedir/queue/gazebo/detector/config.yaml
    else
        queue_det_config_file=$sitedir/queue/detector/config.yaml
    fi

    if [ $queue_det_config_file != "" ]; then
        launch_file="queue_people_py detect_queue_people.launch"
        echo "launch $launch_file"
        eval "$command ros2 launch $launch_file queue_annotation_list_file:=$queue_det_config_file \
                        $commandpost"
        pids+=($!)
    else
        echo "Invalid site is specified. There is no queue config file."
    fi
fi

### obstacle detect/track
if [ $obstacle -eq 1 ]; then
    launch_file="obstacle_convert_tracking.launch"
    echo "launch $launch_file"
    eval "$command roslaunch track_people_cpp $launch_file \
                $commandpost"
    pids+=($!)
fi

## wait until it is terminated by the user
while [ 1 -eq 1 ];
do
    snore 1
done
