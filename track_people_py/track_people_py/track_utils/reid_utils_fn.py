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

import math
import numpy as np


def iou_circles(circle1, circle2):
    # Compute IOU between two circles
    # format circles : [x, y, r]
    # refer equation here : https://diego.assencio.com/?index=8d6ca3d82151bad815f78addf9b5c1c6

    x1, y1, r1 = circle1[:3]
    x2, y2, r2 = circle2[:3]
    dist = np.linalg.norm(np.array([x1, y1]) - np.array([x2, y2]))
    r1_2 = math.pow(r1, 2)
    r2_2 = math.pow(r2, 2)

    if dist < r1 + r2:
        if dist < abs(r1-r2):
            # completely overlap
            intersection = math.pi * min(r1_2, r2_2)
        else:
            dist_2 = math.pow(dist, 2)
            d1 = (r1_2 - r2_2 + dist_2)/(2*dist)
            d1_2 = math.pow(d1, 2)
            d2 = dist - d1
            d2_2 = math.pow(d2, 2)
            intersection = r1_2 * math.acos(d1/r1) - d1 * math.sqrt(r1_2 - d1_2) + r2_2 * math.acos(d2/r2) - d2 * math.sqrt(r2_2 - d2_2)
    else:
        intersection = 0

    union = math.pi*r1_2 + math.pi*r2_2 - intersection

    return intersection / union


def compute_circle_pairwise_iou(circles1, circles2):
    # compute pairwise IOU
    # format circle : [x, y, r]

    iou = np.zeros((len(circles1), len(circles2)))

    for c1 in range(len(circles1)):
        for c2 in range(len(circles2)):
            iou[c1, c2] = iou_circles(circles1[c1], circles2[c2])
            assert iou[c1, c2] >= 0, "{} {} {}".format(iou[c1, c2], c1, c2)

    return iou
