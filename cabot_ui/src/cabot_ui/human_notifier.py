# Copyright (c) 2020 Carnegie Mellon University
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
import numpy, numpy.linalg

from deviation_detector import DeviationDetector
import geoutil, geojson, datautil

class HumanNotifier:
    MIN_INTERVAL = 5
    MIN_DIST = 1.5
    
    def __init__(self, path):
        self.initial_path = path
        self.dev_detector = DeviationDetector(path)
        self.last_notified = rospy.get_rostime().to_sec()        
        self.standing = False

        self.max_dist = 0
        self.angle = 0
        self.pose = None
        self.humans = None
        self.humans_vel = []
        self.dists = []
        self.collisions = []
        self.possible_avoidance = []
        self.speed = 1.0

    def update_path(self, path):
        _, self.max_dist, self.angle = self.dev_detector.check_diff(path)
        
    def update_humans(self, humans):
        self.humans = humans
        self.standing = False
        if humans.circles:
            self.humans_vel = [numpy.linalg.norm([c.velocity.x, c.velocity.y]) for c in self.humans.circles]
            self.speed = self.speed * 0.8 + min(self.humans_vel) * 0.2
        else:
            self.speed = self.speed * 0.8 + 1 * 0.2

        self.standing = self.standing or self.speed < 0.2

    def update_current_pose(self, pose):
        self.pose = pose
        if self.humans is None:
            return
        
        self.dists = [pose.distance_to(geoutil.Point(xy=c.center)) for c in self.humans.circles]
        self.collisions = [self.collision_check(c.center) for c in self.humans.circles]

        self.possible_avoidance = [False]*len(self.dists)
        for i, (d, c) in enumerate(zip(self.dists, self.collisions)):
            if d < HumanNotifier.MIN_DIST and c < 1.0:
                self.possible_avoidance[i] = True

    def collision_check(self, point):
        p1 = numpy.array([point.x, point.y])
        min_dist = 100
        for p in self.initial_path.poses:
            p2 = numpy.array([p.pose.position.x, p.pose.position.y])
            dist = numpy.linalg.norm(p2-p1)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def should_notify(self):
        now = rospy.get_rostime().to_sec()
        result = False
        if (now - self.last_notified > HumanNotifier.MIN_INTERVAL and self.standing
            and self.max_dist > 0.25 and numpy.any(self.possible_avoidance)):
            result = True
            
        if self.humans is not None and self.pose is not None and self.humans.circles:
            if result:
                rospy.loginfo("###############################################################")
            else:
                rospy.loginfo("------------------------------------")
            rospy.loginfo("Since-Last: %.2f", now-self.last_notified)
            rospy.loginfo("Max Dist:   %.2f", self.max_dist)
            rospy.loginfo("Humans:     %d", len(self.humans.circles))
            rospy.loginfo("Humans vel: %s", str(self.humans_vel))
            rospy.loginfo("Standing:   %d", self.standing)
            rospy.loginfo("Speed   :   %.2f", self.speed)
            rospy.loginfo("Dists:      %s", str(self.dists))
            rospy.loginfo("Collisions: %s", str(self.collisions))
            rospy.loginfo("Avoidance : %s", str(self.possible_avoidance))

        return result


    def notified(self):
        self.last_notified = rospy.get_rostime().to_sec()

    def get_angle(self):
        return self.angle
    


