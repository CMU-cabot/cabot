#!/usr/bin/env python3

# Copyright (c) 2021  Carnegine Mellon University and IBM Corporation, and others
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
import rclpy

from detect_abstract_people import AbsDetectPeople

NET_SIZE=416

class DetectDarknetPeople(AbsDetectPeople):    
    __metaclass__ = ABCMeta
    
    def _load_names(self, names_file):
        with open(names_file, 'r') as f:
            classes = f.read().splitlines()
            return classes

    def darknet_load(self, cfg_file, weight_file, names_file):
        try:
            self.get_logger().info("network loading")
            net = cv2.dnn.readNetFromDarknet(cfg_file, weight_file)
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
            model = cv2.dnn_DetectionModel(net)
            model.setInputParams(scale=1 / 255, size=(NET_SIZE, NET_SIZE), swapRB=True)
            self.get_logger().info("network loaded")
            names = self._load_names(names_file)
            return model, names
        except:
            self.get_logger().error("cannot load model file \n{}\n{}\n{}".format(cfg_file, weight_file, names_file))
        

    def __init__(self, device):
        super().__init__(device)
        
        # load detect model
        detect_config_filename = self.declare_parameter('detect_config_file', '').value
        detect_weight_filename = self.declare_parameter('detect_weight_file', '').value
        detect_label_filename = self.declare_parameter('detect_label_file', '').value
        
        self.darknet_net, self.darknet_meta = self.darknet_load(detect_config_filename, detect_weight_filename, detect_label_filename)

    def is_detector_initialized(self):
        if not hasattr(self, 'darknet_net') or not hasattr(self, 'darknet_meta'):
            return False
        return True

    def prepare_image(self, rgb_img):
        return (None, rgb_img)

    def detect_people(self, rgb_img, frame_resized, darknet_image):
        # (classIds, scores, boxes)
        return self.darknet_net.detect(rgb_img, confThreshold=0.6, nmsThreshold=0.4)

    def post_process(self, rgb_img, frame_resized, boxes_res):
        people_res = []
        for idx, score, box in zip(*boxes_res):
            if self.darknet_meta[idx] == "person":
                # convert results to format [xtl, ytl, xbr, ybr, conf, class]
                # 1 is class ID of 'person' class
                xtl = box[0]
                ytl = box[1]
                xbr = box[0] + box[2]
                ybr = box[1] + box[3]
                people_res.append([xtl, ytl, xbr, ybr, score, 1])
        detect_results = np.array(people_res)

        if len(detect_results) > 0:
            # delete small detections
            small_detection = np.where(detect_results[:,3]-detect_results[:,1] < self.minimum_detection_size_threshold)[0]
            detect_results = np.delete(detect_results, small_detection, axis=0)

        return detect_results


def main():
    rclpy.init()
    device = "cuda"

    detect_people = DetectDarknetPeople(device)
    
    try:
        rclpy.spin(detect_people)
    except KeyboardInterrupt:
        detect_people.get_logger().info("Shutting down")


if __name__=='__main__':
    main()
