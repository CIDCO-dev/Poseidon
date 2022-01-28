#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <boost/process.hpp>
#include <boost/process/extend.hpp>
#include <iostream>


TEST(nmeaDeviceTest, testSerialDevice) {
	/*
	//virtual serial port -> sudo apt install socat
	user should be in dialout group
	nmea_device_node listen on /dev/sonar
	sudo ln -s /home/ubuntu/sonar /dev/sonar
	sudo chown -h :dialout /dev/sonar
	once programm started : cat /home/ubuntu/sonar
	*/
	
	int nb_message = 0;
	std::string device;
	boost::process::ipstream output;
	std::string cmd = "socat -d -d pty,raw,echo=0,link=/home/ubuntu/sonar pty,raw,echo=0,link=/home/ubuntu/pty"; 
	boost::process::child virtDev(cmd);
	std::vector<std::string> devices;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	while(virtDev.running()){
		//std::cout<<"running \n";
		
		std::ofstream outfile ("/home/ubuntu/pty",std::ofstream::binary);
		
		if(outfile.is_open()){
		
			outfile << "test\n";
			std::this_thread::sleep_for(std::chrono::seconds(1));
			nb_message++;
			if(nb_message == 10){
				outfile.close();
				virtDev.terminate();
				std::cout<<"terminated \n";
			}
		}
		else{
			std::cerr<<"cannot open virtual port";
		}
	
	}
	
	ASSERT_TRUE(true);
}

int main(int argc, char **argv) {

    //ros::init(argc, argv, "TestNmeaDevices");

    testing::InitGoogleTest(&argc, argv);

    //std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    //ros::shutdown(); // this will cause the ros::spin() to return
    //t.join();

    return res;
}
