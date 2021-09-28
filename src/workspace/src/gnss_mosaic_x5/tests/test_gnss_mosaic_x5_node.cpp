#include <gnss_mosaic_x5/gnss_mosaic_x5.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include <thread>

#include "mosaicX5DatagramCounter.h" // derived class that counts datagram for each type

TEST(PiksiTestSuite, testCaseCountDatagrams) {

	std::string serialPortPath = "/dev/ttyACM0";
	std::string logPath = "";

	MosaicX5DatagramCount mosaicX5(logPath, serialPortPath);
	mosaicX5.run();

	// format in a way that can be ctrl+F in specs
	for( const auto &pair : mosaicX5.datagramCount) {
	    std::stringstream ss;
	    ss << std::hex << pair.first;
	    std::string hexString(ss.str());
        /*
	    if(hexString.length() == 2) {
            hexString = "00" + hexString;
	    } else if(hexString.length() == 3) {
            hexString = "0" + hexString;
	    }
	    */
	    ROS_ERROR_STREAM("Datagram: 0x" << hexString << " count: " << mosaicX5.datagramCount[pair.first]);
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestMosaicNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}