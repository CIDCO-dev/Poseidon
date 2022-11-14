import argparse
# pip install rosbags
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
import sys, os

parser=argparse.ArgumentParser(description="Cidco 2022 {} Rosbag reader".format(os.linesep))
parser.add_argument("rosbag", type=str)
parser.add_argument("-t", "--topics", type=str, nargs='+', action='append', help='list of topic to read')
args = parser.parse_args()
rosbagPath = args.rosbag


if args.topics == None:
	topics = []
else:
	topics = args.topics[0]

if len(topics) == 0:
	pass;
else:
	print("topics to read: ", topics)

# create reader instance and open for reading
with Reader(rosbagPath) as reader:
	for connection, timestamp, rawdata in reader.messages():
		if connection.topic in topics:
			msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
			print(msg.header)
        	
