#include "classes_for_tests.h"

class LoggerBinaryTestSuite : public ::testing::Test {
  public:

    LoggerBinaryTestSuite() {}
    ~LoggerBinaryTestSuite() {}
    
    protected:
    	ros::NodeHandle n;
    	ros::ServiceClient getLoggingStatusServiceClient;
    	ros::ServiceClient toggleLoggingServiceClient;
    	ros::ServiceClient GetLoggingModeServiceClient;
    	ros::ServiceClient SetLoggingModeServiceClient;
    	std::string outPath;
    	
    	GnssSignalGenerator signalGnss;
    	ImuSignalGenerator signalImu;
    	SonarSignalGenerator signalSonar;
    	LidarSignalGenerator signalLidar;
    	std::vector<double> messages;
    	
    	virtual void SetUp() override{
    		this->outPath = "/home/ubuntu/unittestPoseidonRecord";
    		// setup service clients
    		this->getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    		this->toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");
    		this->GetLoggingModeServiceClient = n.serviceClient<logger_service::GetLoggingMode>("get_logging_mode");
    		this->SetLoggingModeServiceClient = n.serviceClient<logger_service::SetLoggingMode>("set_logging_mode");
    		for(double i = 2.0; i<12; i++){
    			messages.push_back(i);
    		}
    		
    	}
    	
    	virtual void TearDown() override{
    		std::filesystem::remove_all(outPath);
    	}
  	
};

TEST_F(LoggerBinaryTestSuite, testGeneratingFiles) {
	
	std::filesystem::create_directory(outPath);
	LoggerBinary logger(outPath);
	
	logger_service::GetLoggingStatus status;
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging should not be enabled";
	
	// logger need fix to enable logging
	signalGnss.publishMessage(0, 1.0, 1.0, 1.0);

    
	// toggle to enable logging
    logger_service::ToggleLogging toggle;
    toggle.request.loggingEnabled = true;
    toggleLoggingServiceClient.call(toggle);
    ASSERT_TRUE(toggle.response.loggingStatus) << "logging callback was not called by service server";
    getLoggingStatusServiceClient.call(status);
    ASSERT_TRUE(status.response.status) << "logging status was not changed after enable toggle";
    
    // try toggle to stop logging
    toggle.request.loggingEnabled = false;
    toggleLoggingServiceClient.call(toggle);
    ASSERT_TRUE(status.response.status) << "logging should not be stoped"; 
    
	
	// because theres no config file, logging mode is set to always on and cannot be toogle off
	logger_service::SetLoggingMode newMode;
	newMode.request.loggingMode = 2; // manual
	SetLoggingModeServiceClient.call(newMode);
	logger_service::GetLoggingMode whatIsMode;
	GetLoggingModeServiceClient.call(whatIsMode);
	ASSERT_TRUE(whatIsMode.response.loggingMode == 2);
	
	// toggle to stop logging
    toggle.request.loggingEnabled = false;
    toggleLoggingServiceClient.call(toggle);
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging status was not changed after enable toggle";
    
    // toggle to enable logging
    toggle.request.loggingEnabled = true;
    toggleLoggingServiceClient.call(toggle);
    ASSERT_TRUE(toggle.response.loggingStatus) << "logging callback was not called by service server";
    getLoggingStatusServiceClient.call(status);
    ASSERT_TRUE(status.response.status) << "logging status was not changed after enable toggle";
    
    
    
    // send messages
    for(auto const &i: messages){
		signalGnss.publishMessage(0, i, i, i);
		sleep(0.5);
		/*
		signalImu.publishMessage(0, i, i, i);
		sleep(0.01);
		*/
    	signalSonar.publishMessage(0, i, i, i);
    	sleep(0.5);
    	
    	std::vector<geometry_msgs::Point32> points;
    	geometry_msgs::Point32 point;
		point.x = float(i); //c++ functional cast
		point.y = float(i);
		point.z = float(i);
		points.push_back(point);
    	sleep(0.5);
    	signalLidar.publishMessage(0, points);
		sleep(0.5);
	}
	sleep(0.5);
    logger.finalize();
	sleep(1.0);

    std::ifstream file;
    std::string filePath;
    std::filesystem::path PATH = outPath;
    for(auto &dir_entry: std::filesystem::directory_iterator{PATH}){
	    if(dir_entry.is_regular_file()){
	    	std::string logFile = std::filesystem::canonical(dir_entry);
	    	if (logFile.find(".log")){
	    		filePath = logFile;
	    	}
	    }
	}
    file.open(filePath, std::ios::out | std::ios::binary);
    ASSERT_TRUE(file.is_open()) << "no file present : " << filePath;
    file.close();
	
    // read file
    PoseidonBinaryReaderTest reader(filePath);
    reader.read();
    
    // file parsed properly
    ASSERT_EQ(reader.getGnssMessagesCount(), static_cast<int>(messages.size())) << reader.getGnssMessagesCount() << "!=" << messages.size();
    ASSERT_EQ(reader.getSonarMessagesCount(), static_cast<int>(messages.size())) << reader.getSonarMessagesCount() << "!=" << messages.size();
    ASSERT_EQ(reader.getLidarMessagesCount(), static_cast<int>(messages.size())) << reader.getLidarMessagesCount() << "!=" << messages.size();
    //ASSERT_TRUE(reader.getImuMessagesCount() == messages.size()) << reader.getImuMessagesCount() << "!=" << messages.size();
    
    // checking gnss file content
    auto positions = reader.getPositions();
    for(size_t i = 0; i<positions.size(); ++i){
    	PositionPacket packet = positions.at(i);
    	sleep(0.01);
    	ASSERT_TRUE(packet.longitude == packet.latitude && packet.latitude == packet.altitude) << "1 parsing problem or binary logger problem for packet position: " << i;
    	sleep(0.01);
    	ASSERT_TRUE(packet.longitude == messages.at(i)) << "2 parsing problem or binary logger problem for packet position: " << i;
    	sleep(0.01);
    }
    
    // checking sonar file content
    auto depths = reader.getDepths();
    for(size_t i = 0; i<depths.size(); ++i){
    	DepthPacket packet = depths.at(i);
    	sleep(0.01);
    	ASSERT_TRUE(packet.depth_x == packet.depth_y && packet.depth_y == packet.depth_z) << "1 parsing problem or binary logger problem for packet depth: " << i;
    	sleep(0.01);
    	ASSERT_TRUE(packet.depth_x == messages.at(i)) << "2 parsing problem or binary logger problem for packet depth: " << i;
    	sleep(0.01);
    }
    
    // checking lidar file content
    auto laserPoints = reader.getLaserPoints();
    for(size_t i = 0; i<laserPoints.size(); ++i){
    	LidarPacket packet = laserPoints.at(i);
    	sleep(0.01);
    	ASSERT_TRUE(packet.laser_x == packet.laser_y && packet.laser_y == packet.laser_z) << "1 parsing problem or binary logger problem for packet lidar: " << i;
    	sleep(0.01);
    	ASSERT_TRUE(packet.laser_x == messages.at(i)) << "2 parsing problem or binary logger problem for packet lidar: " << i;
    	sleep(0.01);
    }
    
    
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "TestLoggerBinary");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
