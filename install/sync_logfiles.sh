#!/bin/sh

USER=$1
DESTINATION_PATH=$2
SERVER=$3
RSA_KEY_FILENAME=$4 # path = /opt/.ssh/
FILES_LOCATION="/opt/Poseidon/www/webroot/record/" # without the last '/' it will transfer the folder instead of files
KEEP_ORIGINAL_FILES=false # set to true for testing purposes

if $KEEP_ORIGINAL_FILES; then
	REMOVE_SOURCE_FILES=""
else
	REMOVE_SOURCE_FILES="--remove-source-files"
fi

# command
rsync -e'ssh -i ~/.ssh/'"$RSA_KEY_FILENAME"'' -apz $FILES_LOCATION $USER@$SERVER:$DESTINATION_PATH $REMOVE_SOURCE_FILES
