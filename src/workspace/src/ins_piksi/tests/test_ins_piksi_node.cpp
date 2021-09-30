#include <ins_piksi/ins_piksi.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include <thread>

#include "piksiOneHourLog.h" // derived class that logs piksi data for one hour
#include "piksiDatagramCounter.h" // derived class that counts datagram for each type

TEST(PiksiTestSuite, testCaseRunOneHour) {

	std::string serialPortPath = "/dev/ttyACM0";
	std::string logPath = "";

	PiksiOneHourLog piksi(logPath, serialPortPath);
	piksi.minutes = 60;
	// Uncomment and wait one hour
	//piksi.run();
}

TEST(PiksiTestSuite, testCaseCountDatagrams) {

	std::string serialPortPath = "/dev/ttyACM0";
	std::string logPath = "";

	PiksiDatagramCount piksi(logPath, serialPortPath);
	piksi.run();

	// format in a way that can be ctrl+F in specs
	for( const auto &pair : piksi.datagramCount) {
	    std::stringstream ss;
	    ss << std::hex << pair.first;
	    std::string hexString(ss.str());

	    if(hexString.length() == 2) {
            hexString = "00" + hexString;
	    } else if(hexString.length() == 3) {
            hexString = "0" + hexString;
	    }
	    ROS_ERROR_STREAM("Datagram: 0x" << hexString << " count: " << piksi.datagramCount[pair.first]);
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestPiksiNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
