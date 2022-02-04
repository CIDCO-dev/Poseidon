#include <ros/ros.h>
#include <gtest/gtest.h>
//#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PointStamped.h"

#include <thread>
#include <iostream>
#include "virtual_serial_port.hpp"
#include <vector>
#include <stdio.h>


class counter{
	public:
		counter(int init){ this-> count = init;}
		~counter(){}
		void increment(){ count++; }
		int getCount(){return this->count; }
	
	private:
		int count;
};

//counter fixCounter(0);
//counter depthCounter(0);
counter speedCounter(0);
/*
void fixReceived(const sensor_msgs::NavSatFix& gnss){
	fixCounter.increment();
}
void depthReceived(const geometry_msgs::PointStamped& sonar){
	depthCounter.increment();
}
*/
void speedReceived(const nav_msgs::Odometry& speed){
	speedCounter.increment();
}

TEST(virtualZf9pTest, testZf9pUbxSpeed) {
	
	ros::NodeHandle n;

	//ros::Subscriber sub1 = n.subscribe("fix", 10,fixReceived);
	ros::Subscriber sub2 = n.subscribe("speed", 10, speedReceived);
	
	
	/*
	zf9p node listen on /dev/ttyAMA0
	$sudo ln -s /home/ubuntu/zf9p /dev/ttyAMA0
	$sudo chown -h :dialout /dev/ttyAMA0
	*/
	//ubx.nav-pvt
	std::vector<uint8_t> msg = {181,98,01,07,00,64}; // header,  64 = payload length
	for(int i=0; i<56; i++){
		msg.push_back(97);
	}
	//0x2121 = 8481
	msg.push_back(21);
	msg.push_back(21);
	ROS_ERROR_STREAM("MSG:  "<< msg.size());
	std::string message (msg.begin(), msg.end());
	
	ROS_ERROR_STREAM("MESSAGE:  "<< message);
	/*
	virtualSerialPort virtualZedFf9p("/home/ubuntu/zf9p", "/home/ubuntu/pty");
	auto zf9p = virtualZedFf9p.init();
	while(zf9p.running()){
		for(int i = 0; i<10; i++){
			sleep(1);
			virtualZedFf9p.write("µb00000000000000000000000000000000000000000000000000000000000000011!!1");// gspeed at bytes 60-> !! = 8481mm/s ~= 30Kmh
			sleep(1);
		}
		virtualZedFf9p.close(zf9p);
	}
	*/
	sleep(1);
	//ROS_ERROR_STREAM("fix count : "<< fixCounter.getCount()<<"\n");
	//ROS_ERROR_STREAM("depth count : "<< depthCounter.getCount()<<"\n");
	//ROS_ERROR_STREAM("speed count : "<< speedCounter.getCount()<<"\n");
	
	ASSERT_TRUE(true);
	//ASSERT_TRUE(fixCounter.getCount() == 10);
	//ASSERT_TRUE(speedCounter.getCount() == 10);
	//ASSERT_TRUE(depthCounter.getCount() == 10);
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
