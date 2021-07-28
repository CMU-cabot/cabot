# Copyright (c) 2020  Carnegie Mellon University
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
from scipy.spatial.distance import euclidean
from matplotlib import pyplot as plt
from fastdtw import fastdtw

class DeviationDetector:
    def __init__(self, path):
        self.path = path
        self.offset = 0

    def check_diff(self, b):
        a = self.path
        
        def convert(c, s, e):
            x = []
            for i in range(s, e):
                if i < len(c.poses):
                    p = c.poses[i]
                    x.append(np.array([p.pose.position.x, p.pose.position.y]))
            return x

        x = convert(a, self.offset, self.offset+300)
        y = convert(b, 0, 300)

        _, path = fastdtw(x, y, dist=euclidean)
            
        dists = []
        for p in path:
            dists.append(euclidean(x[p[0]], y[p[1]]))
                
        s = 0
        for i in range(len(path)-1):
            if path[i][1] != path[i+1][1]:
                s = i
                break
        for i in range(s,len(path)-1):
            if path[i][0] != path[i+1][0]:
                s = i
                break
            
        e = len(path)-1
        for i in range(len(path)-1,1,-1):
            if path[i][0] != path[i-1][0]:
                e = i
                break
        for i in range(e,1,-1):
            if path[i][1] != path[i-1][1]:
                e = i
                break

        if e <= s:
            return 1000, 1000, 0

        max_index = s+np.argmax(dists[s:e])
        max_dist = dists[max_index]
        
        xi = path[max_index][0]
        px = x[xi]
        py = y[path[max_index][1]]
        pd = py - px

        px1 = x[xi-1]
        px2 = x[xi+1]
        pd2 = px2-px1
        
        angle1 = np.arctan2(pd[1], pd[0])
        angle2 = np.arctan2(pd2[1], pd2[0])
        angle = angle1 - angle2

        if angle > np.pi:
            angle = angle - np.pi * 2
        if angle < -np.pi:
            angle = angle + np.pi * 2
            
        dist = np.sum(dists[s:e])

        """
        print "(%4d, %4d)\t(%.2f, %.2f)\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%d"%\
        (s, e, y[0][0], y[0][1], max_dist, dist, angle, angle1, angle2, self.offset)

        if max_dist > 0.25:
            plt.plot(*np.transpose(x[:path[s][0]]), color='red', linestyle=':')
            plt.plot(*np.transpose(y[:path[s][1]]), color='blue', linestyle=':')

            plt.plot(*np.transpose(x[path[s][0]:path[e][0]]), color='red', linestyle='-')
            plt.plot(*np.transpose(y[path[s][1]:path[e][1]]), color='blue', linestyle='-')
            
            plt.plot(*np.transpose(x[path[e][0]:]), color='red', linestyle=':')
            plt.plot(*np.transpose(y[path[e][1]:]), color='blue', linestyle=':')
            
            plt.plot(px[0], px[1], '*')
            plt.plot(py[0], py[1], '+')
            plt.plot(px1[0], px1[1], 'o')
            plt.plot(px2[0], px2[1], 'x')
            plt.show()
        """

        self.offset += s
        return dist, max_dist, angle
