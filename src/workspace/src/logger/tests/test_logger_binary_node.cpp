#include "classes_for_tests.h"
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <sstream>
#include <utility>

namespace {
class EnvVarGuard {
public:
	EnvVarGuard(const char* name, const std::string& newValue) : var(name) {
		const char* current = std::getenv(name);
		if (current) {
			hadOld = true;
			oldValue = current;
		}
		if (newValue.empty()) {
			unsetenv(name);
		} else {
			setenv(name, newValue.c_str(), 1);
		}
	}
	~EnvVarGuard() {
		if (hadOld) {
			setenv(var.c_str(), oldValue.c_str(), 1);
		} else {
			unsetenv(var.c_str());
		}
	}
private:
	std::string var;
	std::string oldValue;
	bool hadOld{false};
};

// Minimal HTTPS server to capture transfer payload locally
class LocalHttpsServer {
public:
	LocalHttpsServer(unsigned short port, const std::string& cert, const std::string& key)
	: port(port), certPem(cert), keyPem(key) {}

	void start() {
		serverThread = std::thread([this]() { run(); });
		while (!ready.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}

	void stop() {
		if (serverThread.joinable()) {
			serverThread.join();
		}
	}

	std::string body() const { return lastBody; }

private:
	void run() {
		using tcp = boost::asio::ip::tcp;
		namespace http = boost::beast::http;

		boost::asio::io_context ioc;
		boost::asio::ssl::context ctx(boost::asio::ssl::context::tlsv12_server);
		ctx.use_certificate_chain(boost::asio::buffer(certPem.data(), certPem.size()));
		ctx.use_private_key(boost::asio::buffer(keyPem.data(), keyPem.size()),
		                    boost::asio::ssl::context::file_format::pem);

		tcp::acceptor acceptor(ioc, {boost::asio::ip::make_address("127.0.0.1"), port});
		ready.store(true);

		tcp::socket socket{ioc};
		acceptor.accept(socket);

		boost::beast::ssl_stream<tcp::socket> stream(std::move(socket), ctx);
		stream.handshake(boost::asio::ssl::stream_base::server);

		boost::beast::flat_buffer buffer;
		http::request<http::string_body> req;
		http::read(stream, buffer, req);
		lastBody = req.body();

		http::response<http::string_body> res{http::status::ok, req.version()};
		res.set(http::field::server, "local-test");
		res.keep_alive(false);
		res.body() = "ok";
		res.prepare_payload();
		http::write(stream, res);

		boost::beast::error_code ec;
		stream.shutdown(ec);
	}

	std::thread serverThread;
	std::atomic<bool> ready{false};
	unsigned short port;
	std::string certPem;
	std::string keyPem;
	std::string lastBody;
};

// Testable subclass to expose transfer configuration
class TestableLoggerBinary : public LoggerBinary {
public:
	using LoggerBinary::LoggerBinary;
	void setTransferConfig(const std::string& host,
	                       const std::string& port,
	                       const std::string& target,
	                       const std::string& apiKey) {
		this->host = host;
		this->port = port;
		this->target = target;
		this->apiKey = apiKey;
		this->activatedTransfer = true;
	}
	void runTransfer() { this->transfer(); }
};

std::pair<std::string, std::string> generateSelfSignedCert() {
	namespace fs = std::filesystem;
	auto tmpDir = fs::temp_directory_path() / "logger_binary_ssl";
	fs::create_directories(tmpDir);

	auto certPath = tmpDir / "cert.pem";
	auto keyPath = tmpDir / "key.pem";
	std::string cmd = "openssl req -x509 -nodes -newkey rsa:2048 -keyout " + keyPath.string() +
	                  " -out " + certPath.string() + " -days 1 -subj \"/CN=127.0.0.1\" >/dev/null 2>&1";
	int rc = std::system(cmd.c_str());
	if (rc != 0) {
		throw std::runtime_error("Failed to generate self-signed certificate for test");
	}
	std::ifstream certStream(certPath);
	std::ifstream keyStream(keyPath);
	std::stringstream certBuf;
	std::stringstream keyBuf;
	certBuf << certStream.rdbuf();
	keyBuf << keyStream.rdbuf();
	return {certBuf.str(), keyBuf.str()};
}

std::size_t currentRssBytes() {
	std::ifstream f("/proc/self/statm");
	long dummy = 0;
	long rssPages = 0;
	if (!(f >> dummy >> rssPages)) {
		return 0;
	}
	long pageSize = sysconf(_SC_PAGESIZE);
	return static_cast<std::size_t>(rssPages) * static_cast<std::size_t>(pageSize);
}
} // namespace

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

TEST_F(LoggerBinaryTestSuite, testToggleRequiresFix) {

	std::filesystem::create_directory(outPath);
	LoggerBinary logger(outPath);

	logger_service::GetLoggingStatus status;
	getLoggingStatusServiceClient.call(status);
	ASSERT_FALSE(status.response.status) << "logging should not be enabled";

	// Toggle on without GNSS fix: should stay disabled
	logger_service::ToggleLogging toggle;
	toggle.request.loggingEnabled = true;
	toggleLoggingServiceClient.call(toggle);
	getLoggingStatusServiceClient.call(status);
	ASSERT_FALSE(status.response.status) << "logging should stay off without GNSS fix";

	// Publish a GNSS fix, then toggling should succeed
	signalGnss.publishMessage(0, 1.0, 1.0, 1.0);
	sleep(1);

	toggleLoggingServiceClient.call(toggle);
	getLoggingStatusServiceClient.call(status);
	ASSERT_TRUE(status.response.status) << "logging should turn on after GNSS fix";

	logger.finalize();
}

TEST_F(LoggerBinaryTestSuite, testTransferToLocalHttpsServer) {

	std::filesystem::create_directory(outPath);

	// Write cert so client (logger) trusts it
	std::string certPath = outPath + "/test_cert.pem";
	auto [cert, key] = generateSelfSignedCert();
	{
		std::ofstream certFile(certPath);
		certFile << cert;
	}
	EnvVarGuard certGuard("SSL_CERT_FILE", certPath);

	const unsigned short port = 9443;
	LocalHttpsServer server(port, cert, key);
	server.start();

	TestableLoggerBinary logger(outPath);
	logger.setTransferConfig("127.0.0.1", std::to_string(port), "/", "test-api-key");

	// Create a dummy zip file to transfer
	std::string zipPath = outPath + "/sample.zip";
	{
		std::ofstream f(zipPath, std::ios::binary);
		f << "dummy";
	}

	logger.runTransfer();

	server.stop();

	ASSERT_FALSE(std::filesystem::exists(zipPath)) << "Zip should be removed after successful transfer";
	ASSERT_FALSE(server.body().empty()) << "Server should have received payload";
	ASSERT_NE(server.body().find("fileData"), std::string::npos) << "Payload should contain fileData";
}

TEST_F(LoggerBinaryTestSuite, testTransferFailuresDoNotGrowMemory) {

	std::filesystem::create_directory(outPath);

	TestableLoggerBinary logger(outPath);
	// Point to a closed port so connects fail fast
	logger.setTransferConfig("127.0.0.1", "65500", "/", "test-api-key");

	// Dummy zip to trigger transfer attempts
	std::string zipPath = outPath + "/sample_failure.zip";
	{
		std::ofstream f(zipPath, std::ios::binary);
		f << "dummy";
	}

	std::size_t before = currentRssBytes();
	for (int i = 0; i < 20; ++i) {
		logger.runTransfer();
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	std::this_thread::sleep_for(std::chrono::seconds(10));
	std::size_t after = currentRssBytes();

	// Allow small fluctuation (2 MB)
	ASSERT_LE(after, before + static_cast<std::size_t>(2 * 1024 * 1024))
	    << "Memory grew after repeated failed transfers";
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
