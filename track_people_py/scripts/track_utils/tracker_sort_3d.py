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

import numpy as np
import matplotlib.pyplot as plt
from . import reid_utils_fn
from . import kf_utils
from scipy.optimize import linear_sum_assignment

class TrackerSort3D:
    def __init__(self,
                 iou_threshold=0.01, iou_circle_size=0.5,
                 minimum_valid_track_duration = 0.3,
                 duration_inactive_to_remove = 2.0,
                 n_colors=100
                ):
        # Initialization
        #
        # iou_threshold : minimum IOU threshold to keep track
        # iou_circle_size : radius of circle in bird-eye view to calculate IOU
        # minimum_valid_track_duration : minimum duration to consider track is valid
        # duration_inactive_to_remove : duration for an inactive detection to be removed
        # n_colors : number of colors to assign to each track
        
        # parameters for Kalman Filter
        self.kf_time_step = 1.0/1.0 # 1 FPS
        self.kf_sigma_proc = 10.0 # process noise : smaller value will be smoother
        self.kf_sigma_meas = 10.0 # measurement noise
        self.sigma_l = 0.0 # if detection confidence score is larger than this value, start tracking
        self.sigma_h = 0.5 # if maximum detection confidence store of track is smaller than this value, finish tracking
        
        # set data keepers
        self.record_tracker = {} # record keeper for active tracks
        self.box_active = {} # dictionary holding the latest box of active detections
        self.kf_active = {} # Kalman Filter for active tracks        
        self.record_tracker_archive = {} # main data storage
        
        # set parameters
        self.iou_threshold = iou_threshold
        self.iou_circle_size = iou_circle_size
        self.minimum_valid_track_duration = minimum_valid_track_duration
        self.duration_inactive_to_remove = duration_inactive_to_remove
        self.n_colors = n_colors
        self.list_colors = plt.cm.hsv(np.linspace(0, 1, n_colors)).tolist() # list of colors to assign to each track for visualization
        np.random.shuffle(self.list_colors) # shuffle colors
        
        # counter of tracks
        self.tracker_count = 0
    
    def track(self, now, bboxes, center_pos_list, frame_id, counter_penalty=1, drop_inactive_feature=True):
        # Performs tracking by comparing with previous detected people
        #
        # INPUT
        # bboxes : n x 4+ matrix - each row is a bbox [x_tl, y_tl, x_br, y_br] 
        #                           of each detection. Additional element (e.g.
        #                           detection score) may be included.
        # center_bird_eye_list : n x 2 matrix - each row is a bbox [x, y] 
        #                           in bird-eye view of each detection. 
        # frame_id : 1 x 1 scalar - id of the frame
        # step_penalty : 1 x 1 scalar - amount to discount the active detection counter
        # drop_inactive_feature : boolean - True to drop features of permanent inactive 
        #                            detection to save space
        #
        # OUTPUT
        # prev_exist : boolean n-vector indicating whether each of the
        #               person exists before
        # person_id : int n-vector indicating id of each person
        # person_color : visualization color of each each person
        # tracked_duration : total tracked duration

        center_circle_list = []
        for center_pos in center_pos_list:
            center_circle_list.append([center_pos[0], center_pos[1], self.iou_circle_size])
        
        # prepare output
        prev_exist = np.zeros(len(bboxes)).astype(np.bool)
        person_id = (-np.ones(len(bboxes))).astype(np.int)
        person_color = [None]*len(bboxes)
        tracked_duration = np.zeros(len(bboxes)).astype(np.float)
        
        if (len(bboxes) == 0) and (len(self.box_active) == 0):
            # No new detection and no active tracks
            # Do nothing
            det_to_add = []
            track_inactive = []
      
            # predict box by Kalman Filter
            for id_track in self.kf_active.keys():
                self.kf_active[id_track]["kf"].predict()
        elif (len(bboxes) == 0) and (len(self.box_active) > 0):
            # No new detection but has active tracks
            
            # no tracks to add
            det_to_add = []
            
            # set all active tracks to inactive
            track_inactive = list(self.box_active.keys())
            
            # predict box by Kalman Filter
            for id_track in self.kf_active.keys():
                self.kf_active[id_track]["kf"].predict()
        elif (len(bboxes) > 0) and (len(self.box_active) == 0):
            
            # If no active detection, add all of them
            det_to_add = np.arange(len(bboxes))
            
            # No track to be considered inactive
            track_inactive = []
            
            # predict box by Kalman Filter
            for id_track in self.kf_active.keys():
                self.kf_active[id_track]["kf"].predict()
        elif (len(bboxes) > 0) and (len(self.box_active) > 0): 
            # If there are active detections, compared them to new detections
            # then decide to match or add as new tracks
            
            # predict circle by Kalman Filter
            kf_pred_circles = []
            for id_track in self.kf_active.keys():
                self.kf_active[id_track]["kf"].predict()
                
                kf_x = self.kf_active[id_track]["kf"].x[0,0]
                kf_y = self.kf_active[id_track]["kf"].x[2,0]
                kf_circle = [kf_x, kf_y, self.iou_circle_size]
                kf_pred_circles.append(kf_circle)
            
            # get all active tracks
            key_box_active = list(self.box_active.keys())
            
            # compute IOU
            iou = reid_utils_fn.compute_circle_pairwise_iou(center_circle_list, kf_pred_circles)

            # match by Hungarian
            row_ind, col_ind = linear_sum_assignment(1-iou)
            
            # get prev and current detection correspondence, then update existing tracks
            track_continue_current = []
            track_continue_prev = []
            for cur_idx, prev_idx in zip(row_ind, col_ind):
               if iou[cur_idx][prev_idx] > self.iou_threshold:
                   track_continue_current.append(cur_idx)
                   track_continue_prev.append(prev_idx)
            track_continue_prev = [key_box_active[i] for i in track_continue_prev] # get id of still active tracks
            
            # Now we have a 1-1 correspondence of active tracks
            # between track id in track_continue_prev and detection in track_continue_current.
            # Add these infos to the record.
            for i, id_track in enumerate(track_continue_prev):
                circle_tmp = center_circle_list[track_continue_current[i]]
                self.record_tracker[id_track]["frame"][frame_id] = {}
                self.record_tracker[id_track]["frame"][frame_id]["bbox"] = bboxes[track_continue_current[i]]
                self.record_tracker[id_track]["expire"] = now + self.duration_inactive_to_remove
                self.box_active[id_track] = circle_tmp
                
                # update Kalman Filter
                kf_meas = [circle_tmp[0], circle_tmp[1]]
                self.kf_active[id_track]["kf"].update(np.asarray(kf_meas).reshape([len(kf_meas), 1]))
                self.kf_active[id_track]["missed"] = 0
                
                # set output
                prev_exist[track_continue_current[i]] = True
                person_id[track_continue_current[i]] = id_track
                person_color[track_continue_current[i]] = self.record_tracker[id_track]["color"]
                tracked_duration[track_continue_current[i]] = (now - self.record_tracker[id_track]["since"]).to_sec()
            
            # the rest of the tracks are new tracks to be add later
            det_to_add = np.setdiff1d(np.arange(len(bboxes)), track_continue_current)
            
            # get the list of tracks that have become inactive
            track_inactive = [x for x in key_box_active if x not in track_continue_prev]
        else:
            assert False, "Something is wrong here... All conditions shouldn't be wrong!"
        
        # deal with inactive tracks
        for id_track in track_inactive:
            
            if now > self.record_tracker[id_track]["expire"] or (now - self.record_tracker[id_track]["since"]) < self.minimum_valid_track_duration: 
                # remove tracks that have been inactive for too long
                # recall that we still have self.record_tracker_archive
                del self.record_tracker[id_track]
                del self.box_active[id_track]
                del self.kf_active[id_track]
        
                
        # add new trackers
        for id_track in det_to_add:
            self.record_tracker[self.tracker_count] = {}
            self.record_tracker[self.tracker_count]["frame"] = {}
            self.record_tracker[self.tracker_count]["frame"][frame_id] = {}
            self.record_tracker[self.tracker_count]["frame"][frame_id]["bbox"] = bboxes[id_track]
            self.record_tracker[self.tracker_count]["id"] = self.tracker_count
            self.record_tracker[self.tracker_count]["color"] = self.list_colors[self.tracker_count % self.n_colors]
            self.record_tracker[self.tracker_count]["expire"] = now + self.duration_inactive_to_remove
            self.record_tracker[self.tracker_count]["since"] = now
            
            # save active box
            self.box_active[self.tracker_count] = bboxes[id_track]
            
            # save active Kalaman Filter
            new_kf_x = center_circle_list[id_track][0]
            new_kf_y = center_circle_list[id_track][1]
            self.kf_active[self.tracker_count] = kf_utils.init_kf_fixed_size([new_kf_x, 0.0, new_kf_y, 0.0], self.kf_time_step, self.kf_sigma_proc, self.kf_sigma_meas)
            
            # save archive data
            self.record_tracker_archive[self.tracker_count] = self.record_tracker[self.tracker_count]
            
            # set output
            prev_exist[id_track] = False
            person_id[id_track]  = self.tracker_count
            person_color[id_track] = self.record_tracker[self.tracker_count]["color"]
            tracked_duration[id_track] = (now - self.record_tracker[self.tracker_count]["since"]).to_sec()
            
            self.tracker_count += 1
        
        return prev_exist, person_id, person_color, tracked_duration
    
    def get_track_id(self, id_track):
        # return track of a given id
        assert (id_track < self.tracker_count) and (id_track >= 0), "Track ID {} is invalid (Total tracks: {})".format(id_track, self.tracker_count)
        return self.record_tracker_archive[id_track]
    
    def get_active_tracks_at_frame(self, id_frame):
        # get IDs, bboxes, and colors of tracks at frame id_frame
        
        id_list = []
        bbox_list = []
        color_list = []
        
        for id_track in range(self.tracker_count):
            
            if id_frame in self.record_tracker_archive[id_track]["frame"]:
                id_list.append(id_track)
                bbox_list.append(self.record_tracker_archive[id_track]["frame"][id_frame]["bbox"])
                color_list.append(self.record_tracker_archive[id_track]["color"])
        
        return id_list, bbox_list, color_list
