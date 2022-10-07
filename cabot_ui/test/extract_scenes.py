#!/usr/bin/env python

import os
import os.path
import argparse
import subprocess
import shutil
import pathlib
import yaml
from cabot_ui.stop_reasonser import StopReasoner
import stop_reasons

def reason_callback(time, code, duration):    
    screencast_extension = pathlib.Path(screencast).suffix
    print(time.to_sec(), code, duration)
    temp = "{}-{}{}".format(str(int(time.to_sec())), str(code), screencast_extension)
    output = os.path.join(basedir, "scenes", temp)
    start = max(0, time.to_sec()-time_sync-args.before-duration)
    duration = duration + args.before + args.after
    extract_scene(screencast, start, duration, output)

def extract_scene(filename, start, duration, output):
    if os.path.exists(output):
        print("file {} exists, so skip it".format(output))
        return

    commands = ["ffmpeg",
                "-ss", str(start),   # start time, should be before input to reduce processing time
                "-i", filename,      # input file
                "-t", str(duration), # duration
                output               # output file
    ]
    print(commands)
    subprocess.run(commands)

def main():
    if args.clear:
        shutil.rmtree(os.path.join(basedir, "scenes"))
    os.makedirs(os.path.join(basedir, "scenes"), exist_ok=True)
    stop_reasons.read_from_bag(bagfile, callback=reason_callback)
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract scenes using yaml')
    parser.add_argument('-m', dest="metadata", metavar='META', help='path to metadata yaml file')
    parser.add_argument('-b', dest="before", type=int, default=3, help='duration (sec) before the scene')
    parser.add_argument('-a', dest="after", type=int, default=3, help='duration (sec) after the scene')
    parser.add_argument('-c', dest="clear", default=False, action='store_true', help='clear scenes folder')

    args = parser.parse_args()

    print(args)
    
    if os.path.exists(args.metadata):
        metadata = yaml.safe_load(open(args.metadata))
        basedir = os.path.dirname(args.metadata)
        bagfile = os.path.join(basedir, metadata['bagfile'])
        screencast = os.path.join(basedir, metadata['screencast']['file'])
        time_sync = metadata['screencast']['time_sync']

    scenes = []
    
    main()

