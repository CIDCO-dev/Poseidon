./src/workspace/devel/lib/imu_dummy/imu_dummy &
IMU=$!
./src/workspace/devel/lib/gnss_dummy/gnss_dummy &
GNSS=$!
./src/workspace/devel/lib/sonar_dummy/sonar_dummy &
SONAR=$!

echo "Press enter to stop"
read DUMMY

kill -9 $IMU
kill -9 $GNSS
kill -9 $SONAR
