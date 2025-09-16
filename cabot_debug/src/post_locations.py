#!/usr/bin/env python3

import json
import logging
import requests
import sys
import time
import traceback
from optparse import OptionParser

from cabot_common.rosbag2 import BagReader

logging.basicConfig(level=logging.INFO)


class PostLocations:
    LOG_API = "api/log"

    def __init__(self):
        self._protocol = "http"
        self._hostname = "localhost:9090/map"

    def get_log_url(self):
        """get the URL for search api"""
        url = F"{self._protocol}://{self._hostname}/{self.LOG_API}"
        return url

    def post_location(self, client, msg):
        log = [{
            'event': 'location',
            'timestamp': int(time.time() * 1000),  # milliseconds
            'latitude': msg.lat,
            'longitude': msg.lng,
            'rotate': msg.global_rotate,
            'client': client,
            'floor': msg.floor
        }]
        data = {
            'action': 'insert',
            'data': json.dumps(log)
        }
        req = requests.post(self.get_log_url(), data=data)
        logging.info(F"post_location {req.status_code} {req.text}")
        if req.status_code != 200:
            raise Exception(F"post_location failed {req.status_code} {req.text}")


parser = OptionParser(usage="""
print/plot topics
Example
{0} -f <bag file> -c cabot_debug -S 10   # post every 10th location message with client=cabot_debug
""".format(sys.argv[0]))

parser.add_option('-f', '--file', type=str, help='bag file to print')
parser.add_option('-s', '--start', type=float, help='start time from the begining', default=0.0)
parser.add_option('-d', '--duration', type=float, help='duration from the start time', default=99999999999999)
parser.add_option('-S', '--skip', type=int, help='skip messages', default=10)
parser.add_option('-c', '--client', type=str, help='client name', default='cabot_debug')

(options, args) = parser.parse_args()

if not options.file:
    parser.print_help()
    sys.exit(0)

logging.info(options)
bagfilename = options.file
reader = BagReader(bagfilename)

reader.set_filter_by_topics(["/cabot/pose_log2"])
reader.set_filter_by_options(options)  # filter by start and duration

poster = PostLocations()
count = 0

while reader.has_next():
    try:
        (topic, msg, t, st) = reader.serialize_next()
    except:
        continue
    if topic == "/cabot/pose_log2":
        count += 1
        if count % options.skip != 0:
            continue
        try:
            poster.post_location(options.client, msg)
        except:  # noqa: #722
            logging.info(poster.get_log_url())
            logging.error(traceback.format_exc())
            sys.exit(1)
