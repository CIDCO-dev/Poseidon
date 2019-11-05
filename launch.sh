# Have to start 'roscore' before calling this script

devel/lib/logger_binary/logger_binary ~/Documents/PoseidonOutput &
LOGGER_BINARY=$!
devel/lib/logger_text/logger_text ~/Documents/PoseidonOutput &
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

