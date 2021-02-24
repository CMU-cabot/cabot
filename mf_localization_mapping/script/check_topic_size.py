#!/usr/bin/env python

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

import sys
import argparse
import rosbag

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("-i","--input_bag", required=True)
  args = parser.parse_args()

  input_bag = args.input_bag

  topic_size_dict = {}
  for topic, msg, time in rosbag.Bag(input_bag, 'r').read_messages(raw=True):
    topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
  topic_size = list(topic_size_dict.items())
  topic_size.sort(key=lambda x: x[1])
  print("topic", "size [GB]")
  for topic, size in topic_size:
    size_gb = size/(1024.0**3)
    print(topic, size_gb)

if __name__ == "__main__":
  main()
