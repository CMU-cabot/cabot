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

from abc import ABCMeta, abstractmethod
import os
import sys
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy

from detect_abstract_people import AbsDetectPeople
# need to load darknet after open3d
from darknet import darknet

def _load_names(names_file):
    with open(names_file) as f:
        names = [line.strip() for line in f if line.strip()]
        return names

def darknet_load(cfg_file, weight_file, names_file):
    rospy.loginfo("network loading")
    net = darknet.load_net_custom(cfg_file.encode("ascii"), weight_file.encode("ascii"), 0, 1)  # batch size = 1
    rospy.loginfo("network loaded")
    names = _load_names(names_file)
    return net, names

class DetectDarknetPeople(AbsDetectPeople):    
    __metaclass__ = ABCMeta
    
    
    def __init__(self, device, detection_threshold, minimum_detection_size_threshold):
        AbsDetectPeople.__init__(self, device, detection_threshold, minimum_detection_size_threshold)
        
        # load detect model
        detect_config_filename = rospy.get_param('track_people_py/detect_config_file')
        detect_weight_filename = rospy.get_param('track_people_py/detect_weight_file')
        detect_label_filename = rospy.get_param('track_people_py/detect_label_file')
        self.darknet_net, self.darknet_meta = darknet_load(detect_config_filename, detect_weight_filename, detect_label_filename)
        self.darknet_image = darknet.make_image(darknet.network_width(self.darknet_net),
                                                darknet.network_height(self.darknet_net), 3)

    def is_detector_initialized(self):
        if not hasattr(self, 'darknet_net') or not hasattr(self, 'darknet_meta'):
            return False
        return True

    def prepare_image(self, rgb_img):
        darknet_image = darknet.make_image(darknet.network_width(self.darknet_net),
                                           darknet.network_height(self.darknet_net), 3)
        frame_resized = cv2.resize(rgb_img,
                                   (darknet.network_width(self.darknet_net),
                                    darknet.network_height(self.darknet_net)),
                                   interpolation=cv2.INTER_LINEAR)
        darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())
        return (frame_resized, darknet_image)

    def detect_people(self, rgb_img, frame_resized, darknet_image):
        return darknet.detect_image(self.darknet_net, self.darknet_meta, darknet_image, thresh=self.detection_threshold, hier_thresh=.5, nms=.45)

    def post_process(self, rgb_img, frame_resized, boxes_res):
        people_res = []
        for idx, box in enumerate(boxes_res):
            if box[0]=="person":
                # convert results to format [xtl, ytl, xbr, ybr, conf, class]
                # 1 is class ID of 'person' class
                xtl = box[2][0] - box[2][2]/2
                ytl = box[2][1] - box[2][3]/2
                xbr = box[2][0] + box[2][2]/2
                ybr = box[2][1] + box[2][3]/2
                people_res.append([xtl, ytl, xbr, ybr, float(box[1]), 1])
        detect_results = np.array(people_res)

        # resize detected boxes to input image size
        if detect_results.shape[0]>0:
            frame_resized_ratio_row = float(rgb_img.shape[0])/float(frame_resized.shape[0])
            frame_resized_ratio_col = float(rgb_img.shape[1])/float(frame_resized.shape[1])
            detect_results[:,[1,3]] *= frame_resized_ratio_row
            detect_results[:,[0,2]] *= frame_resized_ratio_col

        #elapsed_time = time.time() - start_time
        #rospy.loginfo("time for detect :{0}".format(elapsed_time) + "[sec]")

        if len(detect_results) > 0:
            # delete small detections
            small_detection = np.where(detect_results[:,3]-detect_results[:,1] < self.minimum_detection_size_threshold)[0]
            detect_results = np.delete(detect_results, small_detection, axis=0)

        return detect_results


def main():
    device = "cuda"
    detection_threshold = rospy.get_param('track_people_py/detection_threshold')
    # minimum vertical size of box to consider a detection as a track
    minimum_detection_size_threshold = rospy.get_param('track_people_py/minimum_detection_size_threshold')

    detect_people = DetectDarknetPeople(device, detection_threshold, minimum_detection_size_threshold)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("Shutting down")


if __name__=='__main__':
    main()
