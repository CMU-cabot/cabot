#!/usr/bin/python

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
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import subprocess
import signal
import sys
import time
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r1', "--ros-ip-1", required=True, type=str, help='PC 1 IP address')
    parser.add_argument('-r2', "--ros-ip-2", required=True, type=str, help='PC 2 IP address')
    parser.add_argument("-p",  "--pc-type", required=False, type=int, choices=[1, 2], help='PC type : 1 (cabot_ros) or 2 (other)')
    parser.add_argument('-s',  "--cabot-site", required=False, type=str, default="cabot_site_coredo_3d", help='cabot site ID')
    parser.add_argument('-c',  "--compose-timeout", required=False, type=int, default=1000, help='docker compose http timeout')
    parser.add_argument("-r",  "--rmw", required=False, type=str, choices=["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"], default="rmw_cyclonedds_cpp", 
                        help='RMW implementation : rmw_fastrtps_cpp (Fast RTPS) or rmw_cyclonedds_cpp (Cyclone DDS)')
    args = parser.parse_args()
    
    print("PC type=" + str(args.pc_type))
    print("ROS IP 1=" + str(args.ros_ip_1))
    print("ROS IP 2=" + str(args.ros_ip_2))
    print("cabot site=" + str(args.cabot_site))
    print("docker compose timeout=" + str(args.compose_timeout))
    print("RMW implementation=" + str(args.rmw))

    # set environment variables
    env = os.environ.copy()
    env["COMPOSE_HTTP_TIMEOUT"] = str(args.compose_timeout)
    env["CABOT_SITE"] = args.cabot_site
    env["MASTER_IP"] = args.ros_ip_1
    env["RMW_IMPLEMENTATION"] = args.rmw

    cabot_site_path = glob.glob(os.path.join("../cabot_sites", "*", args.cabot_site))
    if not len(cabot_site_path)==1:
        print("Error, unique cabot site path is not found : cabot_site_path = " + str(cabot_site_path))
        sys.exit()
    print("cabot site path=" + str(cabot_site_path[0]))

    # find people simulation or queue simulation config file
    people_sim_config_file = os.path.join(cabot_site_path[0], "people", "gazebo", "simulator", "config.yaml")
    queue_sim_config_file = os.path.join(cabot_site_path[0], "queue", "gazebo", "simulator", "config.yaml")
    if os.path.exists(people_sim_config_file):
        print("people simulation config path=" + str(people_sim_config_file))

        # load people simulation config file
        with open(people_sim_config_file, 'r') as f:
            people_sim_config_data = yaml.load(f)
        
        if "people_sim_config_list" not in people_sim_config_data:
            print("people simulation config file does not have people_sim_config_list key")
            sys.exit()
        
        # get the first simulation config data for setting temporal robot pose
        if len(people_sim_config_data["people_sim_config_list"])==0:
            print("people simulation config file does not have any settings")
            sys.exit()
        people_sim_config_data = people_sim_config_data["people_sim_config_list"][0]

        if "init_frame" not in people_sim_config_data or "frame_z" not in people_sim_config_data["init_frame"]:
            print("people simulation config data does not have frame_z for init_frame : " + str(people_sim_config_data))
            sys.exit()

        env["CABOT_INITZ"] = str(people_sim_config_data["init_frame"]["frame_z"])
    elif os.path.exists(queue_sim_config_file):
        print("queue simulation config path=" + str(queue_sim_config_file))

        # load people simulation config file
        with open(queue_sim_config_file, 'r') as f:
            queue_sim_config_data = yaml.load(f)
        
        if "queue_sim_config_list" not in queue_sim_config_data:
            print("queue simulation config file does not have queue_sim_config_list key")
            sys.exit()
        
        # get the first simulation config data for setting temporal robot pose
        if len(queue_sim_config_data["queue_sim_config_list"])==0:
            print("queue simulation config file does not have any settings")
            sys.exit()
        queue_sim_config_data = queue_sim_config_data["queue_sim_config_list"][0]

        if "init_frame" not in queue_sim_config_data or "frame_z" not in queue_sim_config_data["init_frame"]:
            print("queue simulation config data does not have frame_z for init_frame : " + str(queue_sim_config_data))
            sys.exit()

        env["CABOT_INITZ"] = str(queue_sim_config_data["init_frame"]["frame_z"])
    else:
        print("Warning. There is no people or queue simulation config file, set cabot initial z value as 0.")
        env["CABOT_INITZ"] = str(0.0)

    def sigint_cb(num, frame_id):
        for p_docker in p_docker_list:
            os.killpg(p_docker.pid, signal.SIGINT)
            print "killed docker process ", p_docker
        for p_docker in p_docker_list:
            p_docker.wait()
        exit()
    
    signal.signal(signal.SIGINT, sigint_cb)
    
    # launch docker
    p_docker_list = []
    
    if args.ros_ip_1 == args.ros_ip_2:
        env["ROS_IP"] = args.ros_ip_1

        subp_args = ["docker-compose", "up", "localization", "ros2", "bridge", "people", "ros1"]
        p_docker_1 = subprocess.Popen(subp_args, 
                                    preexec_fn=os.setpgrp,
                                    env=env)
        p_docker_list.append(p_docker_1)
    else:
        if args.pc_type==1:
            env["ROS_IP"] = args.ros_ip_1
            
            subp_args = ["docker-compose", "up", "ros1"]
            p_docker_1 = subprocess.Popen(subp_args, 
                                          preexec_fn=os.setpgrp,
                                          env=env)
            p_docker_list.append(p_docker_1)
        else:
            env["ROS_IP"] = args.ros_ip_2
            
            subp_args = ["docker-compose", "up", "localization", "ros2", "bridge", "people"]
            p_docker_1 = subprocess.Popen(subp_args, 
                                        preexec_fn=os.setpgrp,
                                        env=env)
            p_docker_list.append(p_docker_1)


    time.sleep(365*24*60*60)


if __name__=="__main__":
    main()
