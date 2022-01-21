#!/bin/sh

DESTINATION_PATH="example/path"
SERVER="hostname_or_ip"
USER="Posseidon"
FILES_LOCATION="~/Poseidon/www/webroot/record"
PASSWORD_LESS="poseidon"
KEEP_ORIGINAL_FILES=true

if $KEEP_ORIGINAL_FILES; then
	REMOVE_SOURCE_FILES=""
else
	REMOVE_SOURCE_FILES="--remove-source-files"
fi

rsync -e'ssh -i ~/.ssh/'"$PASSWORD_LESS"'' -apz $FILES_LOCATION $USER@$SERVER:$DESTINATION_PATH $REMOVE_SOURCE_FILES
