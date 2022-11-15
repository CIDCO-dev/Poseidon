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

topics = []

if args.topics == None:
	with Reader(rosbagPath) as reader:
		for connection in reader.connections:
			topics.append(connection.topic)
else:
	topics = args.topics[0]


print("topics to read: ", topics)

# create reader instance and open for reading
with Reader(rosbagPath) as reader:
	for connection, timestamp, rawdata in reader.messages():
		if connection.topic in topics:
			try:
				msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
			except Exception as e:
				print("exception ", repr(e))
			else:
				print(msg, os.linesep)
   	
