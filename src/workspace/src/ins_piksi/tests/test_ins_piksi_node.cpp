#include <ins_piksi/ins_piksi.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include <thread>

class PiksiForTest : Piksi {
public:

    PiksiForTest(std::string & outputFolder, std::string & serialport): Piksi(outputFolder, serialport) {

    }

    void run() {
        openAndInitSerialPort();

	    if(serial_port < 0) {
	        ROS_ERROR_STREAM("Can't run Piksi.");
	        return;
	    }

	    //Wait for GNSS fix to init system time
        bootstrappedGnssTime = true; // for now assume there is fix
        while(!bootstrappedGnssTime){
            sleep(1);
            ros::spinOnce();
        }

        //create and open binary output file
        std::ofstream file;
        std::string outputFilename = outputFolder + std::string(datetime()) + std::string(".sbp");
        file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);

        if(!file) {
            ROS_ERROR_STREAM("Cannot open .sbp file on path: " << outputFilename);
            exit(1);
        } else {
            // align serial port with piksi datagrams
            readUntilAlignedWithDatagrams();

            auto time_start = std::chrono::system_clock::now();
            while( (std::chrono::system_clock::now() - time_start) < std::chrono::minutes{1} ) {
                readAndProcessOneDatagram(serial_port, file);
            }

            close(serial_port);
            file.close();
        }
    }
};

TEST(PiksiTestSuite, testCaseRunOneHour) {

	std::string serialPortPath = "/dev/ttyACM0";
	std::string logPath = "";

	PiksiForTest piksi(logPath, serialPortPath);
	piksi.run();
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
