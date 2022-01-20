#!/bin/sh

DESTINATION_PATH="example/path"
SERVER="hostname_or_ip"
USER="Posseidon"
FILES_LOCATION="~/Poseidon/www/webroot/record"
PASSWORD_LESS="poseidon"

#add condition to add --remove-source-files to rsync command for testing purposes

rsync -e'ssh -i ~/.ssh/'"$PASSWORD_LESS"'' -apz $FILES_LOCATION $USER@$SERVER:$DESTINATION_PATH
