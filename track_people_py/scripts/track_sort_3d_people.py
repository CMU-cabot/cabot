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

import os
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
import numpy as np
import rospy
import time

from track_abstract_people import AbsTrackPeople
from track_utils.tracker_sort_3d import TrackerSort3D


class TrackSort3dPeople(AbsTrackPeople):
    def __init__(self, device, minimum_valid_track_time_length,
                 iou_threshold, iou_circle_size, n_frames_inactive_to_remove):
        AbsTrackPeople.__init__(self, device, minimum_valid_track_time_length)
        
        # set tracker
        self.tracker = TrackerSort3D(iou_threshold=iou_threshold, iou_circle_size=iou_circle_size,
                                     n_frames_min_valid_track_length=minimum_valid_track_time_length,
                                     n_frames_inactive_to_remove=n_frames_inactive_to_remove)
    
    
    def detected_boxes_cb(self, detected_boxes_msg):
        # check if tracker is initialized
        if not hasattr(self, 'tracker'):
            return

        # 2022.01.12: remove time check for multiple detection
        # check if image is received in correct time order
        # cur_detect_time_sec = detected_boxes_msg.header.stamp.to_sec()
        # if cur_detect_time_sec<self.prev_detect_time_sec:
        #    return
        
        # make sure if only one camera is processed
        if self.processing_detected_boxes:
            return
        self.processing_detected_boxes = True
        
        detect_results, center_bird_eye_global_list = self.preprocess_msg(detected_boxes_msg)
        
        start_time = time.time()
        _, id_list, color_list, tracked_length = self.tracker.track(detect_results, center_bird_eye_global_list, self.frame_id)
        elapsed_time = time.time() - start_time
        # rospy.loginfo("time for tracking :{0}".format(elapsed_time) + "[sec]")
        
        self.pub_result(detected_boxes_msg, id_list, color_list, tracked_length)
        
        self.vis_result(detected_boxes_msg, id_list, color_list, tracked_length)
        
        self.frame_id += 1
        # self.prev_detect_time_sec = cur_detect_time_sec
        self.processing_detected_boxes = False


def main():
    device = "cuda"
    
    # minimum valid track length should be always 0 for multi camera
    #minimum_valid_track_time_length = 3 # Minimum time length to consider track is valid
    minimum_valid_track_time_length = 0 # Minimum time length to consider track is valid
    
    iou_threshold = 0.01 # IOU threshold to consider detection between frames as same person
    iou_circle_size = 1.0 # radius of circle in bird-eye view to calculate IOU
    n_frames_inactive_to_remove = 30 # number of frames for a track to be inactive before removal
    
    track_people = TrackSort3dPeople(device, minimum_valid_track_time_length, iou_threshold, iou_circle_size, n_frames_inactive_to_remove)
    
    try:
        plt.ion()
        plt.show()
        rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("Shutting down")


if __name__=='__main__':
    main()
