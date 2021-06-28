#!/bin/bash

 ###############################################################################
 # Copyright (c) 2019  Carnegie Mellon University
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
 ###############################################################################

trap ctrl_c INT QUIT TERM

function ctrl_c() {
    echo "killing all process"
    kill -s 2 0
    snore 3
    exit
}

function snore()
{
    local IFS
    [[ -n "${_snore_fd:-}" ]] || exec {_snore_fd}<> <(:)
    read ${1:+-t "$1"} -u $_snore_fd || :
}

# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros1 environment
source "/opt/ros/$ROS1_DISTRO/setup.bash"

UPLINE=$(tput cuu1)
ERASELINE=$(tput el)

if [ "$1" != 'build' ]; then
    echo "Waiting roscore"
    rosnode list 2> /dev/null
    test=$?
    while [ $test -eq 1 ]; do
	c=$((c+1))
	snore 1
	#echo -n "$UPLINE$ERASELINE"
	echo "Waiting roscore ($c)"
	rosnode list 2> /dev/null
	test=$?
    done

    rosparam load ./bridge_topics_sim.yaml
fi


# unsetting ROS_DISTRO to silence ROS_DISTRO override warning
unset ROS_DISTRO
# setup ros2 environment
source "/opt/overlay_bridge_ws/install/setup.bash"
colcon build
if [ $? != 0 ]; then
    exit
fi

if [ "$1" = "build" ]; then
    exit
fi


source install/local_setup.bash

#ros2 launch nav2_action_bridge bridge_launch.py
ros2 run ros1_bridge parameter_bridge &
#ros2 run custom_bridge custom_bridge &
ros2 run nav2_action_bridge nav2_navigate_to_pose_bridge_node &
ros2 run nav2_action_bridge nav2_navigate_to_pose_bridge_node local/navigate_to_pose &
ros2 run nav2_action_bridge nav2_navigate_through_poses_bridge_node &
ros2 run nav2_action_bridge nav2_navigate_through_poses_bridge_node local/navigate_through_poses &
ros2 run nav2_action_bridge nav2_spin_bridge_node &


source "/opt/ros/$ROS1_DISTRO/setup.bash"

rosparam get /robot_description
res=$?
echo $res
while [ $res -ne 0 ]; do
    rosparam get /robot_description
    res=$?
    echo $res
done
rostopic pub /robot_description std_msgs/String "data: `rosparam get /robot_description`"

while [ 1 -eq 1 ]; do
    snore 1
done
