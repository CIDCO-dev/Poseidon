#include <ros/ros.h>
#include <gtest/gtest.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"

#include <thread>
#include <iostream>
#include <cstdio>
#include "virtual_serial_port.hpp"
#include "sonar_nmea_0183_tcp_client/sonar_nmea_0183_tcp_client.h"
class counter{
	public:
		counter(int init){ this-> count = init;}
		~counter(){}
		void increment(){ count++; }
		int getCount(){return this->count; }
	
	private:
		int count;
};

counter fixCounter(0);
counter depthCounter(0);
counter speedCounter(0);

void fixReceived(const sensor_msgs::NavSatFix& gnss){
	fixCounter.increment();
}
void depthReceived(const geometry_msgs::PointStamped& sonar){
	depthCounter.increment();
}
void speedReceived(const nav_msgs::Odometry& speed){
	speedCounter.increment();
}

TEST(nmeaDeviceTest, testSerialDevice) {

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("fix", 10,fixReceived);
	ros::Subscriber sub3 = n.subscribe("depth", 10, depthReceived);
	ros::Subscriber sub4 = n.subscribe("speed", 10, speedReceived);
	
	/*
	nmea_device_node listen on /dev/sonar by default
	$sudo ln -s /home/ubuntu/sonar /dev/sonar
	$sudo ln -s /dev/pts/0 /home/ubuntu/sonar
	$sudo chown -h :dialout /dev/sonar
	*/

	ros::NodeHandle privateNh("~");
	std::string slaveDevice;
	std::string masterDevice;
	privateNh.param<std::string>("slave_device", slaveDevice, std::string("/tmp/sonar_slave"));
	privateNh.param<std::string>("master_device", masterDevice, std::string("/tmp/sonar_master"));

	// ensure stale symlinks/files don't block socat
	std::remove(slaveDevice.c_str());
	std::remove(masterDevice.c_str());

	virtualSerialPort nmeaDevice(slaveDevice, masterDevice);
	auto sonar = nmeaDevice.init();
	sleep(1);
	while(sonar.running()){
		for(int i = 0; i<5; i++){
			nmeaDevice.write("$GPGGA,133818.75,0100.0000,N,00300.0180,E,1,14,3.1,-13.0,M,-45.3,M,,*52");// fix
			sleep(1);
			nmeaDevice.write("$SDDBT,30.9,f,9.4,M,5.1,F*35"); // depth
			sleep(1);
			nmeaDevice.write("$GPVTG,82.0,T,77.7,M,2.4,N,4.4,K,S*3A"); //speed
			sleep(1);
			//dpt >= 3
			nmeaDevice.write("$INDPT,5.0,0.0,0.0*40"); // depth
			sleep(1);
			//dpt <= 2
			nmeaDevice.write("$INDPT,5.0,0.0*42"); // depth
			sleep(1);
		}
		nmeaDevice.close(sonar);
	}
	/*
	ROS_INFO_STREAM("fix count : "<< fixCounter.getCount()<<"\n");
	ROS_INFO_STREAM("depth count : "<< depthCounter.getCount()<<"\n");
	ROS_INFO_STREAM("speed count : "<< speedCounter.getCount()<<"\n");
	*/
	ASSERT_TRUE(fixCounter.getCount() == 5)<< "fix count : "<< fixCounter.getCount()<<"\n";
	ASSERT_TRUE(speedCounter.getCount() == 5) << "speed count : "<< speedCounter.getCount()<<"\n";
	ASSERT_TRUE(depthCounter.getCount() == 15)<< "speed count : " << speedCounter.getCount()<<"\n";
}


TEST(nmeaDeviceTest, checksum) {
	
	std::string s = "$GPVTG,82.0,T,77.7,M,2.4,N,4.4,K,S*3A";
	
	ASSERT_TRUE(BaseNmeaClient::validateChecksum(s));
	
	s = "$GPVTG,82.0,T,77.7,M,2.4,N,4.4,K,S*FF";
	ASSERT_FALSE(BaseNmeaClient::validateChecksum(s));
}
int main(int argc, char **argv) {

    ros::init(argc, argv, "TestNmeaDevices");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
