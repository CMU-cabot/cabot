#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

import paho.mqtt.client as mqtt
import json
import ssl

class UWBPositioning():

    def __init__(self):

        host = "localhost"
        port = 1883
        topic = "tags"

        # set up ros publisher
        self.pose_pub = rospy.Publisher('uwb_pose', PoseStamped, queue_size=1)

        # set up client to communicate with the tag
        self.client = mqtt.Client()

        # set callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.connect(host, port=port)
        self.client.subscribe(topic)

        self.client.loop_forever()


    def on_connect(self, client, userdata, flags, rc):
        print(mqtt.connack_string(rc))

    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("Subscribed to topic!")

    # callback triggered by a new Pozyx data packet
    def on_message(self, client, userdata, msg):
        msg_data = json.loads(msg.payload.decode())
        rospy.loginfo(msg_data)
        msg_data = msg_data[0]
        if "data" in msg_data.keys():
            info = msg_data["data"]
            coords = info["coordinates"]
            orient = info["orientation"]
            
            # compile ros msg and publish
            ros_msg = PoseStamped()
            ros_msg.header.stamp = rospy.Time.from_seconds(msg_data["timestamp"])
            ros_msg.pose.position.x = coords["x"]
            ros_msg.pose.position.y = coords["y"]
            ros_msg.pose.position.z = coords["z"]
            quat = quaternion_from_euler(orient["roll"], orient["pitch"], orient["yaw"])
            ros_msg.pose.orientation.x = quat[0]
            ros_msg.pose.orientation.y = quat[1]
            ros_msg.pose.orientation.z = quat[2]
            ros_msg.pose.orientation.w = quat[3]
        
            self.pose_pub.publish(ros_msg)

import signal
import sys

def handler(signum, frame):
    signame = signal.Signals(signum).name
    print(f'Signal handler called with signal {signame} ({signum})')
    sys.exit(0)

# Set the signal handler and a 5-second alarm
signal.signal(signal.SIGINT, handler)


if __name__ == '__main__':
    rospy.init_node('uwb_positioning')    
    rospy.loginfo("Starting UWB positioning")
    uwbp = UWBPositioning()

