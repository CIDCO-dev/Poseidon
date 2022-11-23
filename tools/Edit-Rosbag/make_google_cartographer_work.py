import sys
import rosbag
import os
import argparse


parser = argparse.ArgumentParser(description="Cidco 2022 {} Rosbag reader".format(os.linesep))
parser.add_argument("rosbag_input", type=str)
parser.add_argument("rosbag_output", type=str)
parser.add_argument("id", type=int, nargs='+', action='append', help='list of message id, ex: 243 666')
args = parser.parse_args()
inputPath = args.rosbag_input
outputPath = args.rosbag_output

ids = args.id[0]
print("ids: ", ids)

count = 0

with rosbag.Bag(outputPath, 'w') as outbag:
	for topic, msg, timestamp in rosbag.Bag(inputPath).read_messages():
		if topic == "/velodyne_points":
			count += 1
			if count in ids:
				print("continue")
				continue; # skip messages
		
		outbag.write(topic, msg, timestamp)



