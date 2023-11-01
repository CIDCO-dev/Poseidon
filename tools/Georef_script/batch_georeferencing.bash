#!/bin/bash

if [[ $# -ne 3 ]]; then
	echo "usage : bash batch_georeferencing.bash directory sbet_filePath cloud_outputDir"
	exit 1
fi

mkdir -p $3

find $1 -name "*.zip" -exec sh -c 'unzip -d "${1%.*}" "$1"' _ {} \;
find $1 -name "*.log" -print0 | parallel -0 -j3 bash georef_echoboat_lidar.bash $2 {} $3 \;
