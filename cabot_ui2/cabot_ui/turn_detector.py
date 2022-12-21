# Copyright (c) 2020, 2022  Carnegie Mellon University
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

"""
Turn Detector Module
"""

import math
import enum
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

smoothParam = 0.015
lookAheadStepsA = int(2/0.02)
thtAngle0 = 10
thtAngle1 = 1
thtMinimumTurn = 30
thtMinimumDev = 20
bandWidth = 1.0
lookAheadStepsB = int(0.5/0.02)


class Turn:
    class Type(enum.Enum):
        Normal = 1
        Avoiding = 2

    def __init__(self, pose, angle, turn_type=Type.Normal, end=None):
        self.pose = pose
        self.start = pose.pose.position
        self.angle = angle
        self.turn_type = turn_type
        self.end = end
        self.text = ""

        if turn_type == Turn.Type.Normal:
            if angle < -math.pi / 3:
                self.text = F"Turn Right ({angle:.2f})"
            elif angle > math.pi / 3:
                self.text = F"Turn Left ({angle:.2f})"
        elif turn_type == Turn.Type.Avoiding:
            if angle < 0:
                self.text = F"Dev Right ({angle:.2f})"
            elif angle > 0:
                self.text = F"Dev Left ({angle:.2f})"
        else:
            self.text = F"Unknown ({angle:.2f})"

        self.passed = False

    def __str__(self):
        return F"<Turn at ({self.start.x}, {self.start.y}), {self.angle}, type({self.turn_type})>"

    def __repr__(self):
        return self.__str__()


class TurnDetector:
    def __init__(self):
        pass

    @staticmethod
    def detects(path, current_pose=None, visualize=False):
        start = 0
        min_dist = 1000
        if current_pose is not None:
            for p in path.poses:
                dx = p.pose.position.x - current_pose.x
                dy = p.pose.position.y - current_pose.y
                if math.sqrt(dx*dx+dy*dy) < min_dist:
                    min_dist = math.sqrt(dx*dx+dy*dy)
                    min_i = start
                start += 1
        start = min_i
        length = len(path.poses)

        x, y = np.zeros(length), np.zeros(length)
        dx, dy = np.zeros(length), np.zeros(length)
        yaw, dyaw = np.zeros(length), np.zeros(length)
        # dist = np.zeros(length)

        dyaw2 = np.zeros(length)
        TurnStarts, TurnEnds, Angles, Types = [], [], [], []
        TurnB = []

        for i in range(length):
            x[i] = path.poses[i].pose.position.x
            y[i] = path.poses[i].pose.position.y

        # get differential
        dx[1:-1], dy[1:-1] = x[2:]-x[:-2], y[2:]-y[:-2]
        dx[0], dy[0] = x[1]-x[0], y[1]-y[0]
        dx[-1], dy[-1] = x[-1]-x[-2], y[-1]-y[-2]
        for i in range(length):
            yaw[i] = xy2angle(dx[i], dy[i])
            yaw_last = yaw[max(0, i-1)]
            yawi_candidates = (int(yaw_last/360)+np.array(range(-2, 3)))*360+yaw[i]
            arg = np.argmin(abs(yawi_candidates-yaw_last))
            yaw[i] = yawi_candidates[arg]

        # for i in range(length-1):
        #     dist = getDist(x[i], y[i], x[i+1], y[i+1])

        # lowpass filtering
        B, A = signal.iirfilter(3, smoothParam, btype='lowpass')
        yawLP = signal.filtfilt(B, A, yaw)
        dyaw[0:-1] = yawLP[1:]-yawLP[:-1]
        N = 10
        for i in range(0, N):
            dyaw2[0:-N] += abs(dyaw[i:-N+i])/N
        yaw, yawRaw = yawLP, yaw

        # find turns
        i = start
        t_i = 0
        while (i < length-1):
            j = int(min(length-1, i+lookAheadStepsA))

            # if angle difference is bigger than the threshold
            if abs(angDiff(yaw[i], yaw[j])) < thtAngle0:
                # print([i, abs(angDiff(yaw[i],yaw[j]))])
                i += 1
                continue

            else:  # Found a turnStart
                if angDiff(yaw[i], yaw[j]) > 0:  # Right Turn, angle decrease
                    RightTurn = 1
                else:
                    RightTurn = -1

                # Search turnStart - Fine search
                k = j

                # find point that turning starts
                while k > i and abs(dyaw2[k]) > 0.01:
                    k -= 1
                TurnStarts.append(k)

                # Search turnEnd - Fine search
                k = j

                # find point that turning ends
                while k < length-1 and abs(dyaw2[k]) > 0.01:
                    k += 1
                TurnEnds.append(k)

                # if differential is small than minimum turn, ignore

                # ignore very beginning of the path
                # if TurnStarts[-1] < lookAheadStepsA - start:
                #     del TurnStarts[-1]
                #     del TurnEnds[-1]

                # smaller than minimum turn
                if abs(yaw[TurnEnds[-1]]-yaw[TurnStarts[-1]]) < thtMinimumTurn:
                    # smaller than minimum deviation
                    if max(dyaw2[TurnStarts[-1]:TurnEnds[-1]]) < thtMinimumDev / 180 * math.pi:
                        del TurnStarts[-1]
                        del TurnEnds[-1]
                    else:
                        Angles.append(-RightTurn * 180 / 7)
                        Types.append(Turn.Type.Avoiding)
                else:
                    if TurnEnds[t_i] < TurnStarts[t_i]:
                        TurnEnds[t_i], TurnStarts[t_i] = TurnStarts[t_i], TurnEnds[t_i]
                    Angles.append(yaw[TurnEnds[t_i]] - yaw[TurnStarts[t_i]])
                    Types.append(Turn.Type.Normal)
                    t_i += 1
                i = k

        i = 0
        t_i = 0
        while i < length-1:
            j = int(min(length-1, i+lookAheadStepsB))
            if getOffset(x[i], y[i], yaw[i], x[j], y[j]) < bandWidth/2:
                i += 1
                continue
            else:
                TurnB.append(j)
                i = j
                # t_i+=1

        print("*******")
        print(TurnStarts)
        print(TurnEnds)
        print(Angles)
        print(Types)
        print(TurnB)

        turns = []
        for i, j, angle, turn_type in zip(TurnStarts, TurnEnds, Angles, Types):
            sp = path.poses[i]
            sp.header.frame_id = path.header.frame_id
            ep = path.poses[j]
            ep.header.frame_id = path.header.frame_id
            # ang = np.pi/180*(yaw[j]-yaw[i])
            turns.append(Turn(sp, angle, turn_type, ep))

        for turn in turns:
            print(turn.text)

        if visualize:
            TurnDetector._visualize(yaw, x, y, TurnStarts, TurnEnds, yawRaw, yawLP, dyaw, dyaw2)
        return turns

    @staticmethod
    def _visualize(yaw, x, y, TurnStarts, TurnEnds, yawRaw, yawLP, dyaw, dyaw2):
        x_stp = np.array(range(len(yaw)))*0.02

        if 1:  # plot map
            plt.figure(1)
            plt.plot(x, y, 'b-', label='path - top view')
            plt.plot(x[TurnStarts], y[TurnStarts], 'ro', label='TurnStarts')
            plt.plot(x[TurnEnds], y[TurnEnds], 'go', label='TurnEnds')
            # plt.plot(x[TurnB],y[TurnB],'*',label='TurnEnds')
            # plt.legend()
            plt.ylabel('Y (meter)')
            plt.xlabel('X (meter)')

        if 1:  # plot angle theta(degree)
            plt.figure(2)
            # plt.subplot(3,1,1)
            plt.plot(x_stp, yawRaw, '.-', label='yaw')
            plt.plot(x_stp, yawLP, '.-', label='Lowpass yaw')
            plt.plot(x_stp[TurnStarts], yawLP[TurnStarts], 'ro', label='TurnStarts')
            plt.plot(x_stp[TurnEnds], yawLP[TurnEnds], 'go', label='TurnEnds')
            plt.ylabel('orientation (degree)')
            plt.xlabel('Distance(meter)')
            plt.legend()
            # plt.subplot(3,1,2)
            # plt.plot(x_stp,dyaw,'.-')
            # plt.subplot(3,1,3)
            # plt.plot(x_stp,ddyaw,'.-')

        if 1:
            plt.figure(3)
            # plt.plot(dyaw)
            # plt.plot(TurnStarts,dyaw[TurnStarts],'ro',label='TurnStarts')
            # plt.plot(TurnEnds,dyaw[TurnEnds],'go',label='TurnStarts')
            plt.plot(dyaw2)
            plt.plot(TurnStarts, dyaw2[TurnStarts], 'rx', label='TurnStarts2')
            plt.plot(TurnEnds, dyaw2[TurnEnds], 'gx', label='TurnStarts2')

        plt.show()

        if 0:  # plot yaw in frequency domain
            yawFAmp = np.abs(np.fft.fft(yaw))
            wn = 1.0*np.array(range(len(yaw)))/len(yaw)
            plt.plot(wn, yawFAmp, '.', label='.')
            plt.ylabel('amp')
            plt.xlabel('f(in unkown unit)')
            plt.legend()
            plt.show()


def xy2angle(dx, dy):
    if dx == 0:
        dx = 1e-38
    if dy == 0:
        dy = 1e-38
    tht = np.arctan(dy/dx)/np.pi*180
    if dx < 0 and dy > 0:
        tht = tht+180
    if dx < 0 and dy < 0:
        tht = tht+180
    if dx > 0 and dy < 0:
        tht = tht+360
    return tht


def getDist(x0, y0, x1, y1):
    ans = np.sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0))
    # print '(',x0,y0,'), (',x1,y1,') -- ',ans
    return ans


def angDiff(a, b):
    return (a-b+180) % 360 - 180


def getOffset(x0, y0, tht, x1, y1):
    if (tht+45) % 180 < 90:
        return np.cos(np.pi/360*tht) * (y1-y0-(x1-x0)*np.tan(np.pi/360*tht))
    else:
        return np.sin(np.pi/360*tht) * (x1-x0-(y1-y0)/np.tan(np.pi/360*tht))
