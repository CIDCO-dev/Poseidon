import argparse
import rosbag # pip install rosbag
import sys, os

parser=argparse.ArgumentParser(description="Cidco 2022 {} Rosbag cropper".format(os.linesep))
parser.add_argument("rosbag_input", type=str)
parser.add_argument("rosbag_output", type=str)
parser.add_argument("message_start", type=int, help="start after x message")
parser.add_argument("message_end", type=int , help="stop after x message from start")
args = parser.parse_args()
inputPath = args.rosbag_input
outputPath = args.rosbag_output
message_start = args.message_start
message_end = args.message_end

start = 0
end = 0

with rosbag.Bag(outputPath, 'w') as outbag:
	for topic, msg, t in rosbag.Bag(inputPath).read_messages():
		if start < message_start:
			start +=1
		if start >= message_start:
			outbag.write(topic, msg, t)
			end += 1
		if end == message_end:
			break;
