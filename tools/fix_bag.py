import os
import sys
import yaml

bagdir = sys.argv[1]
metadata = os.path.join(bagdir, 'metadata.yaml')


with open(metadata) as file:
    data = yaml.safe_load(file)
    for key in data["rosbag2_bagfile_information"]:
        data["rosbag2_bagfile_information"]["compression_format"] = "zstd"
        data["rosbag2_bagfile_information"]["compression_mode"] = "MESSAGE"

with open(metadata, 'w') as file:
    yaml.dump(data, file)
