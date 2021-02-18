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
import math
import os
import sys
import time
import yaml

import matplotlib.pyplot as plt
import numpy as np
import rosbag
import tf


# for visualization
def rgba2rgb(rgba):
    alphas = rgba[:,3]
    r = np.clip((1 - alphas) * 1.0 + alphas * rgba[:,0], 0, 1)
    g = np.clip((1 - alphas) * 1.0 + alphas * rgba[:,1], 0, 1)
    b = np.clip((1 - alphas) * 1.0 + alphas * rgba[:,2], 0, 1)
    return np.vstack([r,g,b]).T


def visualize(ax, fig, robot_init_position, robot_position_history, robot_orientation_history, people_position_history, 
            robot_goal_position=None, detect_people_success_history=None):
    # clear figure
    ax.cla()

    # draw start, end points
    start_end_points_x = []
    start_end_points_y = []
    start_end_points_x.append(robot_init_position[0])
    start_end_points_y.append(robot_init_position[1])
    if robot_goal_position is not None:
        start_end_points_x.append(robot_goal_position[0])
        start_end_points_y.append(robot_goal_position[1])
    plt.scatter(start_end_points_x, start_end_points_y, s=100, c="k", marker="x")

    # draw robot
    if len(robot_position_history)>0 and len(robot_orientation_history)>0:
        # plot current robot
        arrow_length = 0.2
        arrow_width = 0.1
        facecolor = "r"
        edgecolor = "k"
        yaw = robot_orientation_history[-1][2]
        ax.arrow(robot_position_history[-1][0], robot_position_history[-1][1], arrow_length * math.cos(yaw), arrow_length * math.sin(yaw),
                facecolor=facecolor, edgecolor=edgecolor, head_width=arrow_width, head_length=arrow_width)
        circle = plt.Circle((robot_position_history[-1][0], robot_position_history[-1][1]), arrow_length, color=facecolor, alpha=0.5)
        ax.add_artist(circle)

        # plot robot history
        if len(robot_position_history)>0:
            scatter_size = 80
            robot_history_x = []
            robot_history_y = []
            for position in robot_position_history[:-1]:
                robot_history_x.append(position[0])
                robot_history_y.append(position[1])
            point_color = np.zeros((len(robot_history_x), 4))
            point_color[:,:3] = [1.0, 0.0, 0.0]
            point_color[:,3] = np.linspace(0.2, 1., len(robot_history_x))
            point_color = rgba2rgb(point_color)
            plt.scatter(robot_history_x, robot_history_y, s=scatter_size, c=point_color, marker="o", edgecolors="k")

    # draw people
    if len(people_position_history)>0:
        # plot current people
        circle_size = 0.2
        facecolor = "b"
        for prev_person_position in people_position_history[-1]:
            if detect_people_success_history is None or detect_people_success_history[-1]:
                circle = plt.Circle((prev_person_position[0], prev_person_position[1]), circle_size, ec="k", fc=facecolor, alpha=0.5)
            else:
                circle = plt.Circle((prev_person_position[0], prev_person_position[1]), circle_size, ec="k", fc="w", alpha=0.5)
            ax.add_artist(circle)

        # plot people history
        scatter_size = 80
        if len(people_position_history)>0:
            list_colors = np.zeros((len(people_position_history)-1, 4))
            list_colors[:,:3] = [0.0, 0.0, 1.0]
            list_colors[:,3] = np.linspace(0.2, 1., len(people_position_history)-1)
            list_colors = rgba2rgb(list_colors)

            people_history_x = []
            people_history_y = []
            point_color = []
            for idx, (people_position, color) in enumerate(zip(people_position_history[:-1], list_colors)):
                if detect_people_success_history is None or detect_people_success_history[idx]:
                    for position in people_position:
                        people_history_x.append(position[0])
                        people_history_y.append(position[1])
                        point_color.append(color)
            plt.scatter(people_history_x, people_history_y, s=scatter_size, c=point_color, marker="o", edgecolors="k")
    
            if detect_people_success_history is not None:
                people_history_x = []
                people_history_y = []
                point_color = []
                for idx, (people_position, color) in enumerate(zip(people_position_history[:-1], list_colors)):
                    if detect_people_success_history is not None and detect_people_success_history[idx] is not True:
                        for position in people_position:
                            people_history_x.append(position[0])
                            people_history_y.append(position[1])
                            point_color.append(color)
                plt.scatter(people_history_x, people_history_y, s=scatter_size, c="w", marker="o", edgecolors="k")

    # draw figure
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.grid(True)
    ax.set_aspect('equal', 'datalim')
    fig.canvas.draw()


# for CSV output
def valformat(val):
    if val is None:
        return str(None)
    return "{:.2f}".format(val)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-r',  "--ros-bag", required=True, type=str, help='ROS bag file path')
    parser.add_argument('-t',  "--test-id", required=True, type=int, help='people test ID')
    parser.add_argument('-o',  "--output-csv", required=True, type=str, help='output CSV file (if CSV file already exists, new result will be added.)')
    parser.add_argument('-s',  "--cabot-site", required=False, type=str, default="cabot_site_coredo_3d", help='cabot site ID')
    parser.add_argument("--visualize", required=False, action='store_true', help='visualize evaluation')
    parser.add_argument("--vis-speed", required=False, type=float, default=10.0, help='visualize fast forward speed')
    parser.add_argument("--radius-robot", required=False, type=float, default=0.45, help='radius of robot model')
    parser.add_argument("--radius-person", required=False, type=float, default=0.25, help='radius of person model')
    parser.add_argument("--max-goal-dist", required=False, type=float, default=0.5, help='maximum distance from last robot position to goal position velocity for success')
    parser.add_argument("--min-velocity", required=False, type=float, default=0.1, help='minimum average velocity for success')
    parser.add_argument("--max-dist-ratio", required=False, type=float, default=2.0, help='maximum ratio of moving distance and route distance for success')
    parser.add_argument("--min-dist-person", required=False, type=float, default=2.0, help='minimum distance from robot to person (only distances from robot front/back to person are checked.)')
    parser.add_argument("--thres-close-person", required=False, type=float, default=2.0, help='threshold of distance to count time close person exists around robot')
    args = parser.parse_args()
    
    print("ROS bag file path=" + str(args.ros_bag))
    print("cabot site=" + str(args.cabot_site))
    print("test ID=" + str(args.test_id))
    print("output CSV file path=" + str(args.output_csv))
    if not os.path.exists(args.output_csv):
        with open(args.output_csv, "w") as file:
            file.write("Test ID,Reached goal,Enough fast,Enough short path,Far from person,Time,Route distance,Moving distance,Distance from last pose to goal,Average speed,Ratio of moving distance and route distance,"
                    + "Minimum distance from robot front to person,Minimum distance from robot back to person,Minimum distance from robot right to person,Minimum distance from robot left to person,"
                    + "Time of close person exists\n")
        print("Output CSV file is created in " + args.output_csv + ".")
    else:
        print("Output CSV file already exists in " + args.output_csv + ". New result will be added.")
    output_sim_people_img = os.path.join(os.path.dirname(args.output_csv), str(args.test_id) + "-sim-people.png")
    output_detect_people_img = os.path.join(os.path.dirname(args.output_csv), str(args.test_id) + "-detect-people.png")
    print("output simulation people image path=" + str(output_sim_people_img))
    print("output detect people image path=" + str(output_detect_people_img))
    
    # find people simulation config file
    cabot_site_path = glob.glob(os.path.join("../cabot_sites", "*", args.cabot_site))
    if not len(cabot_site_path)==1:
        print("Error, unique cabot site path is not found : cabot_site_path = " + str(cabot_site_path))
        sys.exit()
    print("cabot site path=" + str(cabot_site_path[0]))

    people_sim_config_file = os.path.join(cabot_site_path[0], "people", "gazebo", "simulator", "config.yaml")
    if not os.path.exists(people_sim_config_file):
        print("Error, people simulation config file does not exist in " + str(people_sim_config_file))
        sys.exit()
    print("people simulation config path=" + str(people_sim_config_file))

    # load people simulation config file
    with open(people_sim_config_file, 'r') as f:
        people_sim_config_data_list = yaml.load(f)
    
    if "people_sim_config_list" not in people_sim_config_data_list:
        print("people simulation config file does not have people_sim_config_list key")
        sys.exit()
    
    # get the simulation config data for specified test ID
    people_sim_config_data = [data for data in people_sim_config_data_list["people_sim_config_list"] if data["test_id"]==args.test_id]
    if len(people_sim_config_data)!=1:
        print("people simulation config file does not have unique config data for test ID " + str(args.test_id))
        sys.exit()
    people_sim_config_data = people_sim_config_data[0]

    robot_init_pose = people_sim_config_data["robot_init_pose"]
    robot_init_position = np.array([robot_init_pose["x"], robot_init_pose["y"]])
    print("robot initial position = " + str(robot_init_position))

    # prepare read from bag file
    bag = rosbag.Bag(args.ros_bag)
    first_time = None
    last_time = None
    start_nav_time = None
    end_nav_time = None
    robot_position_history = []
    robot_orientation_history = []
    robot_goal_position = None
    move_dist = 0.0
    min_dist_robot_front_person = None
    min_dist_robot_back_person = None
    min_dist_robot_right_person = None
    min_dist_robot_left_person = None
    sim_people_position_history = []
    detect_people_success_history = []
    detect_people_position_history = []
    last_people_time = None
    time_close_person_exist = 0.0

    # prepare plot area
    fig = plt.figure(figsize=(20,20))
    ax = fig.add_subplot(111)
    ax_margin = 5.0
    ax.set_xlim(robot_init_position[0]-ax_margin, robot_init_position[0]+ax_margin)
    ax.set_ylim(robot_init_position[1]-ax_margin, robot_init_position[1]+ax_margin)
    plt.ion()
    if args.visualize:
        plt.show()

    # start read from bag file
    for topic, msg, t in bag.read_messages():
        update_plot = False
        last_update_plot_time = None

        if first_time is None:
            first_time = t.to_sec()
        last_time = t.to_sec()

        if topic=="/people_simulator/simulator_status":
            status = msg.status
            print("time = " + str(t.to_sec()) + ", status = " + str(status))
            
            # check navigation status which is defined in SimulatorStatus.msg
            # int8 STATUS_UNKNOWN = 0
            # int8 STATUS_SIMULATOR_INITIALIZED = 1
            # int8 STATUS_ROBOT_READY = 2
            # int8 STATUS_START_ROBOT_MOVE = 3
            # int8 STATUS_FINISH_ROBOT_MOVE = 4
            # int8 STATUS_SIMULATOR_CANCELD = 5
            if status==3:
                start_nav_time = t.to_sec()
            elif status==4 or status==5:
                end_nav_time = t.to_sec()
                break
        elif topic=="/navigate_to_pose/goal":
            robot_goal = msg.goal.pose
            robot_goal_position = np.array([robot_goal.pose.position.x, robot_goal.pose.position.y])
        elif topic=="/people_simulator/robot_pose":
            robot_position = np.array([msg.pose.position.x, msg.pose.position.y])
            robot_orientation = tf.transformations.euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
            print("time = " + str(t.to_sec()) + ", robot position = " + str(robot_position) + ", robot orientation = " + str(robot_orientation))

            # calculate moving distance
            if len(robot_position_history)==0:
                move_dist += np.linalg.norm(robot_position-robot_init_position)
            else:
                move_dist += np.linalg.norm(robot_position-robot_position_history[-1])
            
            robot_position_history.append(robot_position)
            robot_orientation_history.append(robot_orientation)
            update_plot = True
        elif topic=="/people_simulator/people":
            close_people_exists = False
            if len(robot_position_history)>0:
                sim_people_position = []
                for person in msg.people:
                    # calculate distance from robot to person
                    sim_person_position = np.array([person.position.x, person.position.y])
                    dist_robot_person = max(np.linalg.norm(sim_person_position-robot_position_history[-1])-args.radius_robot-args.radius_person, 0.0)
                    
                    if dist_robot_person<args.thres_close_person:
                        close_people_exists = True
                    
                    # calculate orientation from robot to person
                    rad_robot_person = math.atan2(sim_person_position[1]-robot_position_history[-1][1], sim_person_position[0]-robot_position_history[-1][0]) - robot_orientation_history[-1][2]
                    if rad_robot_person<0.0:
                        rad_robot_person += 2.0*math.pi
                    deg_robot_person = 180.0*rad_robot_person/math.pi
                    
                    # calculate minimum distance from each robot side to person
                    if deg_robot_person>=15.0 and deg_robot_person<165.0:
                        print("time = " + str(t.to_sec()) + ", person is left of robot, distance = " + str(dist_robot_person))
                        if min_dist_robot_left_person is None or dist_robot_person<min_dist_robot_left_person:
                            min_dist_robot_left_person = dist_robot_person
                    elif deg_robot_person>=165.0 and deg_robot_person<195.0:
                        print("time = " + str(t.to_sec()) + ", person is back of robot, distance = " + str(dist_robot_person))
                        if min_dist_robot_back_person is None or dist_robot_person<min_dist_robot_back_person:
                            min_dist_robot_back_person = dist_robot_person
                    elif deg_robot_person>=195.0 and deg_robot_person<345.0:
                        print("time = " + str(t.to_sec()) + ", person is right of robot, distance = " + str(dist_robot_person))
                        if min_dist_robot_right_person is None or dist_robot_person<min_dist_robot_right_person:
                            min_dist_robot_right_person = dist_robot_person
                    else:
                        print("time = " + str(t.to_sec()) + ", person is front of robot, distance = " + str(dist_robot_person))
                        if min_dist_robot_front_person is None or dist_robot_person<min_dist_robot_front_person:
                            min_dist_robot_front_person = dist_robot_person
                    print("time = " + str(t.to_sec()) + ", degrees from robot to person = " + str(deg_robot_person))

                    sim_people_position.append(sim_person_position)

                if len(sim_people_position)>0:
                    sim_people_position_history.append(sim_people_position)
                    update_plot = True
            
            if close_people_exists and last_people_time is not None:
                time_close_person_exist += t.to_sec() - last_people_time
            
            last_people_time = t.to_sec()
        elif topic=="/people":
            if len(sim_people_position_history)>0:
                detect_people_position = []
                for person in msg.people:
                    detect_person_position = np.array([person.position.x, person.position.y])
                    detect_people_position.append(detect_person_position)
                if len(detect_people_position)>0:
                    detect_people_position_history.append(detect_people_position)
                    detect_people_success_history.append(True)
                else:
                    detect_people_position_history.append(sim_people_position_history[-1])
                    detect_people_success_history.append(False)
        
        if args.visualize and update_plot:
            visualize(ax, fig, robot_init_position, robot_position_history, robot_orientation_history, sim_people_position_history, robot_goal_position=None)
            if last_update_plot_time is not None:
                time.sleep((t.to_sec()-last_people_time)/args.vis_speed)
            last_update_plot_time = t.to_sec()
    
    # start evaluation
    if start_nav_time is None:
        start_nav_time = first_time
    if end_nav_time is None:
        end_nav_time = last_time

    if len(robot_position_history)>0 and robot_goal_position is not None:
        last_pose_goal_dist = np.linalg.norm(robot_goal_position-robot_position_history[-1])
    else:
        last_pose_goal_dist = None
    route_dist = -1
    if robot_goal_position is not None:
        route_dist = np.linalg.norm(robot_goal_position-robot_init_position)
    move_time = end_nav_time-start_nav_time
    average_velocity = move_dist/move_time
    dist_ratio = move_dist/route_dist
    min_dist_robot_person_list = [x for x in [min_dist_robot_front_person, min_dist_robot_back_person] if x is not None]
    if len(min_dist_robot_person_list)>0:
        min_dist_robot_person = min(min_dist_robot_person_list)
    else:
        min_dist_robot_person = None
    print("move time = " + str(move_time))
    print("route distance = " + str(route_dist))
    print("move distance = " + str(move_dist))
    print("distance from last pose to goal = " + str(last_pose_goal_dist))
    print("average velocity = " + str(average_velocity))
    print("ratio of moving distance and route distance = " + str(dist_ratio))
    print("minimum distance from robot front to person = " + str(min_dist_robot_front_person))
    print("minimum distance from robot back to person = " + str(min_dist_robot_back_person))
    print("minimum distance from robot right to person = " + str(min_dist_robot_right_person))
    print("minimum distance from robot left to person = " + str(min_dist_robot_left_person))
    print("minimum distance from robot front/back to person = " + str(min_dist_robot_person))
    print("time of close people exists around robot = " + str(time_close_person_exist))
    if last_pose_goal_dist is not None and last_pose_goal_dist<args.max_goal_dist:
        result_reach_goal = "success"
    else:
        result_reach_goal = "fail"
    print("reached goal = " + str(result_reach_goal))
    if average_velocity>args.min_velocity:
        result_fast = "success"
    else:
        result_fast = "fail"
    print("enough fast = " + str(result_fast))
    if dist_ratio<args.max_dist_ratio:
        result_short_path = "success"
    else:
        result_short_path = "fail"
    print("enough short path = " + str(result_short_path))
    if min_dist_robot_person is None or min_dist_robot_person>args.min_dist_person:
        result_far_person = "success"
    else:
        result_far_person = "fail"
    print("far from person = " + str(result_far_person))
    
    with open(args.output_csv, "a") as file:
        file.write(str(args.test_id) + "," + result_reach_goal + "," + result_fast + "," + result_short_path + "," + result_far_person + ","
            + valformat(move_time) + "," + valformat(route_dist) + "," + valformat(move_dist) + ","
            + valformat(last_pose_goal_dist) + "," + valformat(average_velocity) + "," + valformat(dist_ratio) + ","
            + valformat(min_dist_robot_front_person) + "," + valformat(min_dist_robot_back_person) + ","
            + valformat(min_dist_robot_right_person) + "," + valformat(min_dist_robot_left_person) + ","
            + valformat(time_close_person_exist) + "\n")

    print("save visualization of simulation people in " + output_sim_people_img)
    visualize(ax, fig, robot_init_position, robot_position_history, robot_orientation_history, sim_people_position_history, 
            robot_goal_position=robot_goal_position, detect_people_success_history=None)
    fig.savefig(output_sim_people_img)

    print("save visualization of detect people in " + output_detect_people_img)
    visualize(ax, fig, robot_init_position, robot_position_history, robot_orientation_history, detect_people_position_history, 
            robot_goal_position=robot_goal_position, detect_people_success_history=detect_people_success_history)
    fig.savefig(output_detect_people_img)


if __name__=="__main__":
    main()
