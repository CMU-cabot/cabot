#!/bin/bash


rosbag filter $1 $1-filtered.bag 'topic in ["", \
                                            "/cmd_vel", \
                                            "/odom", \
                                            "/cabot/event", \
                                            "/cabot/lidar_speed", \
                                            "/cabot/odom_raw", \
                                            "/cabot/people_speed", \
                                            "/cabot/tf_speed", \
					    "/cabot/touch_speed_switched", \
                                            "/cabot/user_speed", \
                                            "/navigate_to_pose/goal", \
                                            "/navigate_to_pose/result", \
                                            "/local/navigate_to_pose/goal", \
					    "/local/navigate_to_pose/result", \
					    "/plan", \
					    "/replan_reason", \
					    "/current_frame", \
					    "/tf", \
					    "/tf_static", \
					    ""]'

