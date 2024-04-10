#!/usr/bin/env python3

import sys
import argparse
import rclpy
from rclpy.topic_endpoint_info import TopicEndpointTypeEnum

parser = argparse.ArgumentParser()
parser.add_argument("-u", "--unmatched", action="store_true", help="output only unmatched items")
args = parser.parse_args()

rclpy.init()
node = rclpy.create_node("qos_check")


def check():
    print(
        ",".join(
            [
                "topic_name",
                "publisher.reliability",
                "subscription.reliability",
                "reliability.match",
                "publisher.durability",
                "subscription.durability",
                "durability.match",
            ]
        )
    )
    for topic_name, topic_type in node.get_topic_names_and_types():
        info_map = {
            TopicEndpointTypeEnum.PUBLISHER: {"reliability": set(), "durability": set()},
            TopicEndpointTypeEnum.SUBSCRIPTION: {"reliability": set(), "durability": set()},
        }
        for info in node.get_publishers_info_by_topic(topic_name):
            if info.endpoint_type == TopicEndpointTypeEnum.INVALID:
                continue

            info_map[info.endpoint_type]["reliability"].update([info.qos_profile.reliability])
            info_map[info.endpoint_type]["durability"].update([info.qos_profile.durability])

        for info in node.get_subscriptions_info_by_topic(topic_name):
            if info.endpoint_type == TopicEndpointTypeEnum.INVALID:
                continue

            info_map[info.endpoint_type]["reliability"].update([info.qos_profile.reliability])
            info_map[info.endpoint_type]["durability"].update([info.qos_profile.durability])

        if (
            not args.unmatched
            or len(info_map[TopicEndpointTypeEnum.PUBLISHER]["reliability"].union(info_map[TopicEndpointTypeEnum.SUBSCRIPTION]["reliability"])) != 1
            or len(info_map[TopicEndpointTypeEnum.PUBLISHER]["durability"].union(info_map[TopicEndpointTypeEnum.SUBSCRIPTION]["durability"])) != 1
        ):
            print(
                ",".join(
                    [
                        topic_name,
                        "|".join(map(str, info_map[TopicEndpointTypeEnum.PUBLISHER]["reliability"])),
                        "|".join(map(str, info_map[TopicEndpointTypeEnum.SUBSCRIPTION]["reliability"])),
                        str("" if len(info_map[TopicEndpointTypeEnum.PUBLISHER]["reliability"].union(info_map[TopicEndpointTypeEnum.SUBSCRIPTION]["reliability"])) == 1 else "NG"),
                        "|".join(map(str, info_map[TopicEndpointTypeEnum.PUBLISHER]["durability"])),
                        "|".join(map(str, info_map[TopicEndpointTypeEnum.SUBSCRIPTION]["durability"])),
                        str("" if len(info_map[TopicEndpointTypeEnum.PUBLISHER]["durability"].union(info_map[TopicEndpointTypeEnum.SUBSCRIPTION]["durability"])) == 1 else "NG"),
                    ]
                )
            )
    timer.cancel()
    sys.exit(0)


timer = node.create_timer(1, check)
try:
    rclpy.spin(node)
except:
    pass
