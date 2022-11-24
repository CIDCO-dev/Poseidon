import argparse
# pip install rosbags
import rosbag # pip install rosbag
import sys, os

parser=argparse.ArgumentParser(description="Cidco 2022 {} Rosbag topic extracter".format(os.linesep))
parser.add_argument("rosbag_input", type=str)
parser.add_argument("rosbag_output", type=str)
parser.add_argument("topics", type=str, nargs='+', action='append', help='list of topic to extract, ex: /fix /laserScan')
args = parser.parse_args()
inputPath = args.rosbag_input
outputPath = args.rosbag_output
topics = args.topics[0]
print("topics to extract: ", topics)

with rosbag.Bag(outputPath, 'w') as outbag:
	for topic, msg, t in rosbag.Bag(inputPath).read_messages():
		if topic in topics:
			outbag.write(topic, msg, t)
   	
