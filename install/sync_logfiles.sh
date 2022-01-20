#!/bin/sh
rsync -e'ssh -i ~/.ssh/private_key' -ap '~/Poseidon/www/webroot/record' user@server: destination/folder
