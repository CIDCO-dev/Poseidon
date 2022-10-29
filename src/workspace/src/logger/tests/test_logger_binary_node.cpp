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
    	std::vector<double> gnssMessages;
    	
    	virtual void SetUp() override{
    		this->outPath = "/home/ubuntu/unittestPoseidonRecord";
    		// setup service clients
    		this->getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    		this->toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");
    		this->GetLoggingModeServiceClient = n.serviceClient<logger_service::GetLoggingMode>("get_logging_mode");
    		this->SetLoggingModeServiceClient = n.serviceClient<logger_service::SetLoggingMode>("set_logging_mode");
    		for(double i = 2.0; i<12; i++){
    			gnssMessages.push_back(i);
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
    for(auto const &i: gnssMessages){
		signalGnss.publishMessage(0, i, i, i);
		sleep(0.1);
	}
    logger.finalize();
    
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
    //sleep(0.1);
    ASSERT_TRUE(reader.getGnssMessagesCount() == gnssMessages.size()) << reader.getGnssMessagesCount() << "!=" << gnssMessages.size();
    auto positions = reader.getPositions();
    for(int i = 0; i<positions.size(); ++i){
    	PositionPacket packet = positions.at(i);
    	sleep(0.01);
    	ASSERT_TRUE(packet.longitude == packet.latitude && packet.latitude == packet.altitude) << "1 parsing problem or binary logger problem for packet position: " << i;
    	sleep(0.01);
    	ASSERT_TRUE(packet.longitude == gnssMessages.at(i)) << "2 parsing problem or binary logger problem for packet position: " << i;
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
