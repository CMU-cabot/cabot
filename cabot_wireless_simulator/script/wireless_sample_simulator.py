#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022  IBM Corporation
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

import argparse
import json

import numpy as np

from sklearn.neighbors import NearestNeighbors

class SampleSimulator:
    def __init__(self, samples):
        self.samples = samples
        self.area_floor_const = 10000

        X = []
        for s in samples:
            info = s["information"]
            x = info["x"]
            y = info["y"]
            z = info["z"]
            floor_num = info["floor_num"]
            X.append([x,y,z, float(floor_num)*self.area_floor_const])

        X = np.array(X)
        nn = NearestNeighbors(n_neighbors=1)
        nn.fit(X)
        self.nn = nn

    def simulate(self, x, y, z, floor):
        x_query = [[x, y, z, float(floor)*self.area_floor_const]]
        dists, indices = self.nn.kneighbors(x_query)
        index = indices[0,0]
        s = self.samples[index]
        return s["data"]["beacons"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="input sample json file")
    args = parser.parse_args()

    input_file = args.input

    with open(input_file) as f:
        samples = json.load(f)

    sample_simulator = SampleSimulator(samples)

    simulated_data = sample_simulator.simulate(0, 0, 0, 1)
    print(simulated_data)
    