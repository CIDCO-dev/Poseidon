#!/bin/sh


DESTINATION_PATH="destination_Path"
SERVER="hostname or ip"
USER="HB_XX"
FILES_LOCATION="/home/ubuntu/Poseidon/www/webroot/record/" # without the last '/' it will transfer the folder instead of files
PASSWORD_LESS="test"
KEEP_ORIGINAL_FILES=false # set to true for testing purposes

if $KEEP_ORIGINAL_FILES; then
	REMOVE_SOURCE_FILES=""
else
	REMOVE_SOURCE_FILES="--remove-source-files"
fi

rsync -e'ssh -i ~/.ssh/'"$PASSWORD_LESS"'' -apz $FILES_LOCATION $USER@$SERVER:$DESTINATION_PATH $REMOVE_SOURCE_FILES
