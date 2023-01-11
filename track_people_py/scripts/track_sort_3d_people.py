#!/usr/bin/env python3

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

import signal
import sys

from matplotlib import pyplot as plt
import rclpy
from rclpy.duration import Duration
import time

from track_people_py import AbsTrackPeople
from track_people_py.track_utils import TrackerSort3D
from track_people_msgs.msg import TrackedBoxes


class TrackSort3dPeople(AbsTrackPeople):
    def __init__(self, device, minimum_valid_track_duration,
                 iou_threshold, iou_circle_size, duration_inactive_to_remove):
        super().__init__(device, minimum_valid_track_duration)

        # set tracker
        self.tracker = TrackerSort3D(iou_threshold=iou_threshold, iou_circle_size=iou_circle_size,
                                     minimum_valid_track_duration=Duration(seconds=minimum_valid_track_duration),
                                     duration_inactive_to_remove=Duration(seconds=duration_inactive_to_remove))

        self.combined_detected_boxes_pub = self.create_publisher(TrackedBoxes, 'people/combined_detected_boxes', 10)

        self.buffer = {}

    def detected_boxes_cb(self, detected_boxes_msg):
        self.get_logger().info("detected_boxes_cb")
        # check if tracker is initialized
        if not hasattr(self, 'tracker'):
            return

        # 2022.01.12: remove time check for multiple detection
        # check if image is received in correct time order
        # cur_detect_time_sec = detected_boxes_msg.header.stamp.to_sec()
        # if cur_detect_time_sec<self.prev_detect_time_sec:
        #    return

        if detected_boxes_msg.camera_id not in self.buffer:
            self.buffer[detected_boxes_msg.camera_id] = detected_boxes_msg
            return

        # make sure if only one camera is processed
        if self.processing_detected_boxes:
            return
        self.processing_detected_boxes = True

        combined_msg = None

        for key in self.buffer:
            msg = self.buffer[key]
            if not combined_msg:
                combined_msg = msg
            else:
                combined_msg.tracked_boxes.extend(msg.tracked_boxes)
        self.buffer = {detected_boxes_msg.camera_id: detected_boxes_msg}

        detect_results, center_bird_eye_global_list = self.preprocess_msg(combined_msg)

        self.combined_detected_boxes_pub.publish(combined_msg)

        start_time = time.time()
        stamp = rclpy.time.Time.from_msg(detected_boxes_msg.header.stamp)
        try:
            _, id_list, color_list, tracked_duration = self.tracker.track(stamp, detect_results, center_bird_eye_global_list, self.frame_id)
        except:
            self.get_looger().error("tracking error")
            self.processing_detected_boxes = False
            return
        elapsed_time = time.time() - start_time
        # self.get_looger().info("time for tracking :{0}".format(elapsed_time) + "[sec]")

        self.pub_result(combined_msg, id_list, color_list, tracked_duration)

        self.vis_result(combined_msg, id_list, color_list, tracked_duration)

        self.frame_id += 1
        # self.prev_detect_time_sec = cur_detect_time_sec
        self.processing_detected_boxes = False


def main():
    rclpy.init()
    device = "cuda"

    # minimum valid duration should be always 0 for multi camera
    # minimum_valid_track_duration = rospy.Duration(0.3) # Minimum duration to consider track is valid
    minimum_valid_track_duration = 0  # Minimum duration to consider track is valid

    iou_threshold = 0.01  # IOU threshold to consider detection between frames as same person
    iou_circle_size = 1.0  # radius of circle in bird-eye view to calculate IOU
    duration_inactive_to_remove = 2.0  # duration (seconds) for a track to be inactive before removal

    track_people = TrackSort3dPeople(device, minimum_valid_track_duration, iou_threshold, iou_circle_size, duration_inactive_to_remove)

    plt.ion()
    plt.show()
    rclpy.spin(track_people)


def receiveSignal(signal_num, frame):
    print("Received:", signal_num)
    sys.exit(0)


signal.signal(signal.SIGINT, receiveSignal)

if __name__ == '__main__':
    main()
