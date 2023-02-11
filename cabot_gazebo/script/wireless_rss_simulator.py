#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021, 2022  IBM Corporation, Carnegie Mellon University, and others
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

import numpy as np


class BLEBeacon:
    def __init__(self, uuid, major, minor, x, y, z, floor, power):
        self._uuid = uuid
        self._major = int(major)
        self._minor = int(minor)
        self._x = float(x)
        self._y = float(y)
        self._z = float(z)
        self._floor = float(floor)
        self._power = float(power)

    def __str__(self):
        return ",".join([self._uuid, str(self._major), str(self._minor),
                        str(self._x), str(self._y), str(self._z), str(self._floor), str(self._power)])

    def __repr__(self):
        return self.__str__()


class Beacon:
    def __init__(self, uuid, major, minor, rss):
        self._uuid = uuid
        self._major = int(major)
        self._minor = int(minor)
        self._rss = float(rss)

    def __str__(self):
        return ",".join([self._uuid, str(self._major), str(self._minor), str(self._rss)])

    def __repr__(self):
        return self.__str__()


class SimpleBLESimulator:
    def __init__(self, blebeacons, parameters=None, verbose=False):

        self._blebeacons = blebeacons
        self._verbose = verbose

        if parameters is None:
            self._n = 2.0
            self._A = -71
            self._fa = 15.0
            self._fb = 4.0

            self._rmin = 1.0
            self._sigma = 4.0
        else:
            self._n = parameters["n"]
            self._A = parameters["A"]
            self._fa = parameters["fa"]
            self._fb = parameters["fb"]

            self._rmin = parameters["rmin"]
            self._sigma = parameters["sigma"]

    def simulate(self, x, y, z, floor, cutoff=None):
        bs = []
        for ble in self._blebeacons:
            distance = np.sqrt((x-ble._x)**2 + (y-ble._y)**2 + (z-ble._z)**2)
            distance = distance if self._rmin < distance else self._rmin
            rss = -10.0*self._n*np.log10(distance) + self._A + ble._power
            # floor attenuation
            if int(floor) != int(ble._floor):
                diff_floor = np.abs(floor - ble._floor)
                rss_attenuation = self._fa + self._fb*(diff_floor - 1)
                rss = rss - rss_attenuation

            rss += self._sigma * np.random.randn()

            b = Beacon(ble._uuid, ble._major, ble._minor, rss)
            bs.append(b)

        if cutoff is not None:
            bs_filtered = []
            for b in bs:
                if b._rss > cutoff:
                    bs_filtered.append(b)
            bs = bs_filtered

        return bs


def read_csv_blebeacons(blebeacons_file):
    blebeacons = []
    with open(blebeacons_file) as f:
        line = f.readline().strip()  # header
        keys = line.split(",")
        line = f.readline().strip()  # beacon 0
        while line != "":
            tokens = line.split(",")
            beacon_dict = {}
            for i, key in enumerate(keys):
                beacon_dict[key] = tokens[i]

            power = 0
            if "power" in beacon_dict.keys():
                power = beacon_dict["power"]
                try:
                    power = float(power)
                except ValueError:
                    power = 0.0

            blebeacon = BLEBeacon(beacon_dict["uuid"], beacon_dict["major"], beacon_dict["minor"],
                                  beacon_dict["x"], beacon_dict["y"], beacon_dict["z"], beacon_dict["floor"],
                                  power
                                  )
            blebeacons.append(blebeacon)

            line = f.readline().strip()

    return blebeacons


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", required=True, help="input csv file of BLE beacons")
    parser.add_argument("--index", default=0, type=int,  help="index of BLE beacon to simulate RSS")
    args = parser.parse_args()

    blebeacons_file = args.input
    idx = args.index

    blebeacons = read_csv_blebeacons(blebeacons_file)
    ble_simulator = SimpleBLESimulator(blebeacons)

    point = blebeacons[idx]

    beacons = ble_simulator.simulate(point._x, point._y, point._z, point._floor)

    for b in beacons:
        print(b)
