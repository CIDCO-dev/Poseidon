import sys
import datetime

if len(sys.argv) != 2:
	sys.stderr.write("Usage: veryfy_log.py path-to-text-log-file\n")
	sys.exit(1)

filename = sys.argv[1]

sys.stderr.write(f"Parsing {filename}\n")

lineCount = 0
lastTimestamp = False

with open(filename) as logFile:
	for line in logFile:
		splitLine = line.split(";")

		# ignore incomplete lines
		if lineCount > 0  and len(splitLine) == 2:
			# extract timestamp in format 2023-10-27 19:28:29.000000
			currentTimestamp = datetime.datetime.strptime(splitLine[0],"%Y-%m-%d %H:%M:%S.%f")

			if lastTimestamp:
				timeDifference = currentTimestamp - lastTimestamp

				if timeDifference.total_seconds() != 1:
					print("Sonar data is not every 1 seconds")
					sys.exit(1)

			lastTimestamp = currentTimestamp

		lineCount = lineCount + 1

print("Sonar data OK")
