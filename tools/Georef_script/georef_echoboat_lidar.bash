#!/bin/bash

rosrun logger lidarGeoreferencer -f -a -135 -b -45 -d 1 -e 50 -h 180 -p 45 -x 0.04667 -y -0.0362 -z -0.3473 -s $1 $2 > $3'/'`basename $2`'.xyz'
