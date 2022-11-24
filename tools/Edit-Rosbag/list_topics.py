import argparse
# pip install rosbags
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import sys, os

parser=argparse.ArgumentParser(description="Cidco 2022 {} Rosbag topic lister".format(os.linesep))
parser.add_argument("rosbag", type=str)
args = parser.parse_args()
rosbagPath = args.rosbag

with Reader(rosbagPath) as reader:
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)
        	
