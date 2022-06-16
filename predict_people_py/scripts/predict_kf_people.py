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

import rospy
import math
from geometry_msgs.msg import Point
from people_msgs.msg import People, Person
from matplotlib import pyplot as plt

from predict_kf_abstract import PredictKfAbstract 

class PredictKfPeople(PredictKfAbstract):
    def __init__(self, input_time, output_time, duration_inactive_to_remove, duration_inactive_to_stop_publish, fps_est_time, publish_simulator_people):
        PredictKfAbstract.__init__(self, input_time, output_time, duration_inactive_to_remove, duration_inactive_to_stop_publish, fps_est_time)

        # start initialization
        self.publish_simulator_people = publish_simulator_people

        # set subscriber, publisher
        if self.publish_simulator_people:
            self.simulator_people = None
            self.simulator_people_sub = rospy.Subscriber('people_simulator/people', People, self.simulator_people_cb, queue_size=1)

    
    def simulator_people_cb(self, msg):
        self.simulator_people = msg.people
    
    
    def pub_simulator_people(self, msg):
        if self.simulator_people is None:
            return
        simulator_people_msg = People()
        simulator_people_msg.header = msg.header
        simulator_people_msg.people = self.simulator_people
        self.people_pub.publish(simulator_people_msg)
    
    
    def pub_result(self, msg, alive_track_id_list, track_pos_dict, track_vel_dict, track_vel_hist_dict):
        self.htd.tick()
        # init People message
        people_msg = People()
        people_msg.header = msg.header
        
        for track_id in track_pos_dict.keys():
            past_center3d = Point()
            past_center3d.x = track_pos_dict[track_id][0]
            past_center3d.y = track_pos_dict[track_id][1]
            past_center3d.z = 0.0

            # create Person message
            person = Person()
            person.name = str(track_id)
            person.position = past_center3d
            person.velocity = Point()
            person.velocity.x = track_vel_dict[track_id][0]
            person.velocity.y = track_vel_dict[track_id][1]
            person.velocity.z = 0.0
            if track_id in alive_track_id_list:
                person.reliability = 1.0
            else:
                person.reliability = 0.9

            tvel = 0
            tvelcount = 1
            enough_duration = False
            for i in range(-1, -len(track_vel_hist_dict[track_id]), -1):
                (timestamp, vel) = track_vel_hist_dict[track_id][i]
                if (msg.header.stamp.to_sec() - timestamp) > 2.0:
                    enough_duration = True
                    break
                tvel += math.sqrt(vel[0]*vel[0]+vel[1]*vel[1])
                tvelcount += 1

            if enough_duration and tvelcount > 0 and tvel / tvelcount < 0.1:
                person.tags.append("stationary")
            else:
                if "stationary" in person.tags:
                    person.tags.remove("stationary")

            people_msg.people.append(person)

        # merge people from multiple camera before publish
        # self.camera_id_people_dict[msg.camera_id] = copy.copy(people_msg.people)
        # for camera_id in self.camera_id_people_dict.keys():
        #    if camera_id!=msg.camera_id and len(self.camera_id_people_dict[camera_id])>0:
        #        people_msg.people.extend(self.camera_id_people_dict[camera_id])
        self.people_pub.publish(people_msg)
    
    def on_tracked_boxes_cb(self, msg):
        if self.publish_simulator_people:
            self.pub_simulator_people(msg)
            return
    
def main():
    rospy.init_node('predict_people_py', anonymous=True)

    publish_simulator_people = rospy.get_param("~publish_simulator_people")

    input_time = 5 # number of frames to start prediction
    output_time = 5 # number of frames to predict
    duration_inactive_to_remove = 2.0 # duration (seconds) for a track to be inactive before removal (this value should be enough long because track_people_py resturns recovered tracks)
    duration_inactive_to_stop_publish = 0.2 # duration (seconds) for a track to be inactive before stop publishing in people topic
    fps_est_time = 100 # number of frames which are used to estimate FPS
    
    predict_people = PredictKfPeople(input_time, output_time, duration_inactive_to_remove, duration_inactive_to_stop_publish, fps_est_time, publish_simulator_people)
    
    try:
        plt.ion()
        plt.show()
        rospy.spin()
    except KeyboardInterrupt:
      rospy.loginfo("Shutting down")


if __name__=='__main__':
    main()
