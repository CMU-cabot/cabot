#!/usr/bin/python3


import sys
import rclpy.node
import rclpy.logging

from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

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

        logger.info("bag_file=%s"%(bag_file))
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
                log_counter+=1

                if len(stack) > 20:
                    sys.exit()

                if stack[-1] is None or stack[-1].node_name != e.node_name:
                    if not is_final_state(e) and e.node_name != "NavigateWithReplanning":
                        print("[%.2f]%s%s"%(Time.from_msg(e.timestamp).nanoseconds/1e9, " "*len(stack), e.node_name))
                        stack.append(e)
                else: # same node name
                    prev = stack.pop()
                    diff = (Time.from_msg(e.timestamp) - Time.from_msg(prev.timestamp)).nanoseconds/1e9
                    print("[%.2f]%s%s %.2f"%(Time.from_msg(e.timestamp).nanoseconds/1e9, " "*len(stack), e.node_name, diff))

                #print("S[%6d] %10d.%10d %30s %10s %10s"%(log_counter, e.timestamp.sec, e.timestamp.nanosec, e.node_name, e.previous_status, e.current_status))                    
                
'''
                if e.node_name == "AvoidPeople":
                    if e.current_status != "SUCCESS":
                        #print("S[%6d] %10d.%10d %30s %10s %10s"%(log_counter, e.timestamp.sec, e.timestamp.nanosec, e.node_name, e.previous_status, e.current_status))
                        prev = e
                    else:
                        diff = (Time.from_msg(e.timestamp) - Time.from_msg(prev.timestamp)).nanoseconds/1e9
                        print("[%6d] %10d.%10d %30s %6.3f"%(log_counter, e.timestamp.sec, e.timestamp.nanosec, e.node_name, diff))
'''

def main():
    rclpy.init()
    node = CheckLog()
    node.start()

if __name__ == '__main__':
    main()
