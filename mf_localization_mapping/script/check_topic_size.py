#!/usr/bin/env python3

# Copyright (c) 2021, 2023  IBM Corporation and Carnegie Mellon University
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
import rosbag2_py


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input_bag", required=True)
    args = parser.parse_args()

    storage_options, converter_options = get_rosbag_options(args.input_bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_size_dict = {}
    while reader.has_next():
        (topic, msg, time) = reader.read_next()
        topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg)
    topic_size = list(topic_size_dict.items())
    topic_size.sort(key=lambda x: x[1])
    print("topic", "size [GB]")
    for topic, size in topic_size:
        size_gb = size/(1024.0**3)
        print(topic, size_gb)


if __name__ == "__main__":
    main()
