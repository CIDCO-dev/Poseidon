#!/bin/bash

if [[ $# -ne 2 ]]; then
	echo "usage : bash lidar_calib_print.bash inputDir cloud_outputDir"
	exit 1
fi

mkdir -p $3

#find $1 -name "*.zip" -exec sh -c 'unzip -d "${1%.*}" "$1"' _ {} \;

VAR=$(find $1 -name "*.log" -type f)
FILES=($VAR)
#echo $FILES

for item in "${FILES[@]}"
do
	echo $item
	rosrun logger calib_printer $item $2 > $3'/'`basename $item`'.lidar'
done
