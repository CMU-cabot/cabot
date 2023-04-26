#!/usr/bin/python3


import sys
import rclpy.node
import rclpy.logging

from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def is_final_state(e):
    return e.current_status != "RUNNING"


logger = rclpy.logging.get_logger("check_log")


class CheckLog(rclpy.node.Node):
    def __init__(self):
        super().__init__('check_log_py_node')

        self.declare_parameter('bag_file', '')

    def start(self):
        bag_file = self.get_parameter('bag_file').get_parameter_value().string_value

        logger.info("bag_file=%s" % (bag_file))
        storage_options, converter_options = get_rosbag_options(bag_file)

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        storage_filter = rosbag2_py.StorageFilter(topics=['/behavior_tree_log'])
        reader.set_filter(storage_filter)

        log_counter = 0

        prev = None

        stack = [None]

        while reader.has_next():
            (topic, data, time) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            for i, e in enumerate(msg.event_log):
                log_counter += 1
                print("[%.2f]%s%s (%s-%s)" % (Time.from_msg(e.timestamp).nanoseconds/1e9, " "*len(stack), e.node_name, e.previous_status, e.current_status))


def main():
    rclpy.init()
    node = CheckLog()
    node.start()


if __name__ == '__main__':
    main()
