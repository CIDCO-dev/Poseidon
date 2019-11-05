# It is necessary to start 'roscore' before calling this script

# Parameter to provide in the command line: The folder path where to put 
# the output files from logger_binary and logger_text
# e.g. 
# ./launch.sh ~/Documents/PoseidonOutput


devel/lib/logger_binary/logger_binary "$1" &
LOGGER_BINARY=$!

devel/lib/logger_text/logger_text "$1" &
LOGGER_TEXT=$!

devel/lib/imu_dummy/imu_dummy &
IMU=$!
devel/lib/gnss_dummy/gnss_dummy &
GNSS=$!
devel/lib/sonar_dummy/sonar_dummy &
SONAR=$!

echo ""
echo "Press enter to stop"
read DUMMY

kill -9 $IMU
kill -9 $GNSS
kill -9 $SONAR
kill -9 $LOGGER_BINARY
kill -9 $LOGGER_TEXT

