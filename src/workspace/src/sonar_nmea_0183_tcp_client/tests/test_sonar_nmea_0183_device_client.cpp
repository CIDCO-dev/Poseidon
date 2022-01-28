#include <ros/ros.h>
#include <gtest/gtest.h>

#include <thread>
#include <boost/process.hpp>
#include <boost/process/extend.hpp>
#include <iostream>

class virtualSerialPort{
	public:
		virtualSerialPort(std::string Slave, std::string Master){
			this->slave = Slave;
			this->master = Master;
		}
		~virtualSerialPort(){}
		
		
		boost::process::child init(){
		/*
		//virtual serial port -> sudo apt install socat
		user should be in dialout group
		nmea_device_node listen on /dev/sonar
		sudo ln -s /home/ubuntu/sonar /dev/sonar
		sudo chown -h :dialout /dev/sonar
		once programm started : cat /home/ubuntu/sonar
		*/
			//TODO , handle errors
			std::string cmd = "socat -d -d pty,raw,echo=0,link="+ this->slave + " pty,raw,echo=0,link=" + this->master;
			boost::process::child virtDev(cmd);
			std::this_thread::sleep_for(std::chrono::seconds(1));
			return virtDev;
		}
		
		void write(std::string message){
			//TODO , handle errors
			std::ofstream outfile (this->master, std::ofstream::binary);
			if(outfile.is_open()){
				outfile << message << "\r\n";
			}
			else{
				std::cerr<<"cannot open master";
			}
		}
		
		void close(boost::process::child &virtDev){
			virtDev.terminate();
		}
		
		
	private:
		std::string slave;
		std::string master;
		//boost::process::child virtDev;
};




TEST(nmeaDeviceTest, testSerialDevice) {
	
	virtualSerialPort nmeaDevice("/home/ubuntu/sonar", "/home/ubuntu/pty");
	auto sonar = nmeaDevice.init();
	while(sonar.running()){
		for(int i = 0; i<10; i++){
			nmeaDevice.write("test");
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		nmeaDevice.close(sonar);
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
