#!/usr/bin/env python3

# Copyright (c) 2020, 2022  Carnegie Mellon University
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

import os
import time
import fcntl
import subprocess
import traceback
import threading
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
import traceback
from std_msgs.msg import String

BUFFER_SIZE=1000000


def enqueue_output(out, queue):
    '''
    For non-blocking pipe output reading
    '''
    buffer = bytearray()
    count = 0
    while True:
        try:
            r = os.read(out.fileno(), BUFFER_SIZE)
        except OSError:
            time.sleep(0.001)
            count += 1
            if count > 2 and len(buffer) > 0:
                queue.put(buffer.decode('utf-8'))
                buffer = bytearray()
                count = 0
        except:
            time.sleep(0.001)
            node.get_logger().error(traceback.format_exc(), throttle_duration_sec=1)
        else:
            if len(r) == 0:
                break
            count = 0
            for c in r:
                buffer += c.to_bytes(1, byteorder='big')
                if c == '\n':
                    queue.put(buffer.decode('utf-8'))
                    buffer = bytearray()
    out.close()


def commandLoggerNode():
    node.get_logger().info("CABOT ROS Command Logger Node")

    command = node.declare_parameter("command", None).value
    topic = node.declare_parameter("topic", None).value
    frequency = node.declare_parameter("frequency", 0).value
    wait_duration = node.declare_parameter("wait", 0.1).value

    if command is None:
        node.get_logger().error("command should be specified")
    if topic is None:
        node.get_logger().error("topic should be specified")
    if command is None or topic is None:
        return

    node.get_logger().info("command: {}".format(command))
    node.get_logger().info("topic: {}".format(topic))
    node.get_logger().info("frequency: {}".format(frequency))

    pub = node.create_publisher(String, topic, 10)

    try:
        # for non interactive process
        if frequency > 0:
            rate = node.create_rate(frequency)
            while rclpy.ok:
                proc = subprocess.Popen(command,
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE,
                                        shell=True,
                                        env={"COLUMNS": "1000"})

                buffer = bytearray()
                while rclpy.ok:
                    try:
                        r = os.read(proc.stdout.fileno(), BUFFER_SIZE)
                    except OSError:
                        time.sleep(0.001)
                        count += 1
                    except:
                        time.sleep(0.001)
                        node.get_logger().error(traceback.format_exc(), throttle_duration_sec=1)
                    else:
                        if len(r) == 0:
                            break
                        count = 0
                        for c in r:
                            buffer += c.to_bytes(1, byteorder='big')

                msg = String()
                msg.data = buffer.decode('utf-8')
                pub.publish(msg)
                node.get_logger().info("publish: %d", len(buffer))
                rate.sleep()

        # for interactive process
        else:
            proc = subprocess.Popen(command,
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE,
                                    shell=True,
                                    env={"COLUMNS": "1000"}
                                    )
            queue = Queue()
            # make proc.stoudout to non blocking
            flags = fcntl.fcntl(proc.stdout, fcntl.F_GETFL)
            fcntl.fcntl(proc.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)

            thread = threading.Thread(target=enqueue_output,
                                      args=(proc.stdout, queue))
            thread.daemon = True
            thread.start()

            last_time = time.time()
            buffer = ""
            while True:
                if not rclpy.ok or not thread.is_alive():
                    break
                try:
                    line = queue.get_nowait()
                except Empty:
                    if time.time() - last_time > wait_duration \
                       and len(buffer) > 0:
                        msg = String()
                        msg.data = buffer.strip()
                        node.get_logger().info("publish: {}".format(len(msg.data)))
                        pub.publish(msg)
                        buffer = ""
                        last_time = time.time()
                    try:
                        time.sleep(0.001)
                    except:
                        break
                else:
                    if len(buffer) == 0:
                        # node.get_logger().info("start reading")
                        pass
                    buffer += line
                    last_time = time.time()

    except:
        node.get_logger().error(traceback.format_exc())


if __name__ == "__main__":
    rclpy.init()
    node = Node("command_logger_node")
    commandLoggerNode()