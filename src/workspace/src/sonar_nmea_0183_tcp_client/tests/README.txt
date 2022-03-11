
make sure /home/ubuntu/Poseidon/www/webroot/record/ is created or whatever the path is in the launch file

sudo apt install socat

nmea_device_node listen on /dev/sonar by default
$sudo ln -s /home/ubuntu/sonar /dev/sonar
$sudo ln -s /dev/pts/0 /home/ubuntu/sonar
$sudo chown -h :dialout /dev/sonar
