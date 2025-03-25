#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
import traceback

class CabotEventMapper(Node):
    SYNC = 0
    SYNC_START = 1
    SYNC_STOP = 2
    SYNC_DISCARD = 3
    SPEECH_EXCUSE_ME = 1

    def __init__(self):
        super().__init__('cabot_event_mapper')
        self.subscription = self.create_subscription(
            String,
            '/cabot/event',
            self.listener_callback,
            10)
        self.sync_publisher = self.create_publisher(Int16, '/sync_command', 10)
        self.speech_publisher = self.create_publisher(Int16, '/speech_command', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received event: {msg.data}')

        if msg.data == 'click_1_1':  # forward
            self.send_sync_stop()
        if msg.data == 'holddown_1_1':  # forward hold 1 sec
            self.send_sync()
        if msg.data == 'click_2_1':  # backward
            self.send_sync_start()
        if msg.data == 'click_3_1':  # left
            self.send_speech_command()
        if msg.data == 'click_4_1':  # right
            self.send_sync_discard()

    def send_sync(self):
        sync_msg = Int16()
        sync_msg.data = CabotEventMapper.SYNC
        self.sync_publisher.publish(sync_msg)

    def send_sync_start(self):
        sync_msg = Int16()
        sync_msg.data = CabotEventMapper.SYNC_START
        self.sync_publisher.publish(sync_msg)

    def send_sync_stop(self):
        sync_msg = Int16()
        sync_msg.data = CabotEventMapper.SYNC_STOP
        self.sync_publisher.publish(sync_msg)

    def send_sync_discard(self):
        sync_msg = Int16()
        sync_msg.data = CabotEventMapper.SYNC_DISCARD
        self.sync_publisher.publish(sync_msg)

    def send_speech_command(self):
        speech_msg = Int16()
        speech_msg.data = CabotEventMapper.SPEECH_EXCUSE_ME
        self.speech_publisher.publish(speech_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CabotEventMapper()
    try:
        rclpy.spin(node)
    except Exception as e:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
