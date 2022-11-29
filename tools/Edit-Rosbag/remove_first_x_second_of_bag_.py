import argparse
import rosbag # pip install rosbag
import sys, os

parser=argparse.ArgumentParser(description="Cidco 2022 {} Remove first x seconds of bag".format(os.linesep))
parser.add_argument("rosbag_input", type=str)
parser.add_argument("rosbag_output", type=str)
parser.add_argument("offset", type=int, help='seconds')
args = parser.parse_args()
inputPath = args.rosbag_input
outputPath = args.rosbag_output
offset = args.offset

times = [0,0]
# create reader instance and open for reading
with rosbag.Bag(outputPath, 'w') as outbag:
	for topic, msg, timestamp in rosbag.Bag(inputPath).read_messages():
		
		if times[0] == 0:
			times[0] = timestamp.to_sec()
		
		times[1] = timestamp.to_sec()
		
		if not times[1] - times[0] < offset:
			outbag.write(topic, msg, timestamp)
   	
