#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

import argparse
import json
import yaml
import copy

def main():
    parser = argparse.ArgumentParser("extract floor map informaton from a ros map yaml file")
    parser.add_argument("-i","--input", required=True, nargs="*")
    args = parser.parse_args()

    input_yaml_files = args.input
    print(input_yaml_files)

    for yaml_file in input_yaml_files:
        print(yaml_file)

        with open(yaml_file, "r") as f:
            yml = yaml.safe_load(f)

            resolution = yml["resolution"]
            ppm = 1.0/resolution

            origin_x =  -yml["origin"][0]*ppm
            origin_y =  -yml["origin"][1]*ppm

            print("origin_x: "+str(origin_x))
            print("origin_y: "+str(origin_y))
            print("ppm: "+str(ppm))

if __name__ == "__main__":
    main()
