# Copyright (c) 2020  Carnegie Mellon University
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

import os.path
import rclpy
import sys
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from nav2_msgs.srv import LoadMap
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticStatus

PACKAGE_PREFIX = 'package://'


def get_filename(url):
    mod_url = url
    if url.find(PACKAGE_PREFIX) == 0:
        mod_url = url[len(PACKAGE_PREFIX):]
        pos = mod_url.find('/')
        if pos == -1:
            raise Exception("Could not parse package:// format into file:// format for "+url)

        package = mod_url[0:pos]
        mod_url = mod_url[pos:]
        package_path = get_package_share_directory(package)

        mod_url = package_path + mod_url
    return mod_url


def map_filename_callback(msg):
    global current_map_filename, needs_update
    if current_map_filename != msg.data:
        current_map_filename = msg.data
        needs_update = True


def check_update():
    global needs_update, error_message
    if not needs_update:
        return

    filename = get_filename(current_map_filename)
    g_node.get_logger().info(filename)

    if not os.path.exists(filename):
        error_message = "{} is not found".format(filename)
        return

    servers = g_node.get_parameter('map_servers').get_parameter_value().string_array_value

    g_node.get_logger().info(str(servers))

    for server in servers:
        cli = g_node.create_client(LoadMap, server+'/load_map')
        req = LoadMap.Request()
        req.map_url = filename  # need to specify file path not url

        if cli.wait_for_service(timeout_sec=10.0):
            g_node.get_logger().info(server + ' service is ready')
            cli.call_async(req)
        else:
            error_message = server + ' service is not available'
            g_node.get_logger().info(server + ' service is not available')
            return

    needs_update = False


error_message = None


def check_status(stat):
    if error_message:
        stat.summary(DiagnosticStatus.ERROR, error_message)
        return stat
    stat.summary(DiagnosticStatus.OK, "working")
    return stat


def main(args=None):
    global g_node, current_map_filename, needs_update

    current_map_filename = None
    needs_update = False
    rclpy.init(args=args)

    g_node = rclpy.create_node('map_loader')

    g_node.declare_parameter('map_servers', ['/map_server'])

    subscription = g_node.create_subscription(
        String, 'current_map_filename', map_filename_callback, 10)

    updater = Updater(g_node)
    updater.add("ROS2 Map Loader", check_status)

    timer = g_node.create_timer(1.0, check_update)

    g_node.get_logger().info("map_loader is launched")
    try:
        rclpy.spin(g_node)
    except:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    g_node.destroy_node()
    # rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
