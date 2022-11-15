import argparse
# pip install rosbags
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import sys, os
parser=argparse.ArgumentParser(description="Cidco 2022 {} Rosbag message counter".format(os.linesep))
parser.add_argument("rosbag", type=str)

args = parser.parse_args()
rosbagPath = args.rosbag

counter = 0
# create reader instance and open for reading
with Reader(rosbagPath) as reader:
	for connection, timestamp, rawdata in reader.messages():
		counter+=1

print("message counter = ", counter)
