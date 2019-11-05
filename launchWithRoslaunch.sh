# Parameter to provide in the command line: The folder path where to put 
# the output files from logger_binary and logger_text
# e.g. 
# ./launchWithRoslaunch.sh  ~/Documents/PoseidonOutput

source devel/setup.bash

roslaunch src/workspace/launch/hydroball.launch loggerPath:="$1"

#roslaunch src/workspace/launch/hydroball.launch loggerPath:="/home/cidco-hydroball/Documents/PoseidonOutput"

