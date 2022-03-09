#!/bin/sh


DESTINATION_PATH="path"
SERVER="ip / hostname"
USER="HB_XX"
FILES_LOCATION="/home/ubuntu/Poseidon/www/webroot/record/" # without the last '/' it will transfer the folder instead of files
RSA_KEY_FILENAME="Private Key" # path = /home/ubuntu/.ssh/
KEEP_ORIGINAL_FILES=false # set to true for testing purposes

if $KEEP_ORIGINAL_FILES; then
	REMOVE_SOURCE_FILES=""
else
	REMOVE_SOURCE_FILES="--remove-source-files"
fi

# command
rsync -e'ssh -i ~/.ssh/'"$RSA_KEY_FILENAME"'' -apz $FILES_LOCATION $USER@$SERVER:$DESTINATION_PATH $REMOVE_SOURCE_FILES
