#include <ros/ros.h>
#include <gtest/gtest.h>

#include <fstream> 
#include <filesystem>
#include <unistd.h>
#include "loggerText.h"
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

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

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
class TestableLoggerText : public LoggerText {
public:
	using LoggerText::LoggerText;
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
	auto tmpDir = fs::temp_directory_path() / "logger_text_ssl";
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

class LoggerTextTestSuite : public ::testing::Test {
  public:

    LoggerTextTestSuite() {}
    ~LoggerTextTestSuite() {}
    
    protected:
		ros::NodeHandle n;
		ros::ServiceClient getLoggingStatusServiceClient;
		ros::ServiceClient toggleLoggingServiceClient;
		ros::ServiceClient GetLoggingModeServiceClient;
		ros::ServiceClient SetLoggingModeServiceClient;
		std::string outPath;
    	
    	virtual void SetUp() override{
    		this->outPath = "/home/ubuntu/unittestPoseidonRecord";
    		// setup service clients
    		this->getLoggingStatusServiceClient = n.serviceClient<logger_service::GetLoggingStatus>("get_logging_status");
    		this->toggleLoggingServiceClient = n.serviceClient<logger_service::ToggleLogging>("toggle_logging");
    		this->GetLoggingModeServiceClient = n.serviceClient<logger_service::GetLoggingMode>("get_logging_mode");
    		this->SetLoggingModeServiceClient = n.serviceClient<logger_service::SetLoggingMode>("set_logging_mode");
    	}
    	
    	virtual void TearDown() override{
    		std::filesystem::remove_all(outPath);
    	}
  	
};

TEST_F(LoggerTextTestSuite, testGeneratingFiles) {

	std::filesystem::create_directory(outPath);
	LoggerText logger(outPath);
	
	logger_service::GetLoggingStatus status;
    getLoggingStatusServiceClient.call(status);
    ASSERT_FALSE(status.response.status) << "logging should not be enabled";
	
	// wait for gnss dummy node
	sleep(2);
    
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
    logger.finalize();
    
    int numberOfFiles = 0;
    std::filesystem::path PATH = outPath;
    std::string gpsFile;
		for(auto &dir_entry: std::filesystem::directory_iterator{PATH}){
		    if(dir_entry.is_regular_file()){
//		    	std::string temp = std::filesystem::canonical(dir_entry);
//		    	if (temp.find("_gnss.txt")){
//		    		gpsFile = temp;
//		    	}
		    	numberOfFiles++;
		    }
		}
    
    ASSERT_TRUE(numberOfFiles == 4);
    
    
//    int numberOfLines = 0;
//    std::string line;
//    std::ifstream myfile(gpsFile);

//    while (std::getline(myfile, line)){
//    	numberOfLines++;
//    }
//    ROS_ERROR_STREAM ("Number of lines in text file: " << numberOfLines);
//    ASSERT_TRUE(numberOfLines >=7); //on slower pc number of lines might be lower
	
    //clean folder
    std::filesystem::path PATH2 = outPath;
    for(auto &dir_entry: std::filesystem::directory_iterator{PATH2}){
	    if(dir_entry.is_regular_file()){
		    numberOfFiles--;
		    std::filesystem::remove(dir_entry);
	    }
	}
    ASSERT_TRUE(numberOfFiles == 0);
    
}

TEST_F(LoggerTextTestSuite, testWritingAndMovingFiles) {

	std::filesystem::create_directory(outPath);
	LoggerText logger(outPath);

	GnssSignalGenerator gnss;
	SonarSignalGenerator sonar;
	LidarSignalGenerator lidar;

	// Bootstrap GNSS fix to allow logging
	gnss.publishMessage(0, 1.0, 2.0, 3.0);
	sleep(1);

	// Enable logging
	logger_service::ToggleLogging toggle;
	toggle.request.loggingEnabled = true;
	toggleLoggingServiceClient.call(toggle);

	// Publish some data to populate files
	gnss.publishMessage(1, 4.0, 5.0, 6.0);
	sonar.publishMessage(1, 7.0, 8.0, 9.0);
	std::vector<geometry_msgs::Point32> points;
	geometry_msgs::Point32 point;
	point.x = 1.0f;
	point.y = 2.0f;
	point.z = 3.0f;
	points.push_back(point);
	lidar.publishMessage(1, points);
	sleep(1);

	logger.finalize();

	int numberOfFiles = 0;
	std::string gnssFilePath;
	for (auto &dir_entry : std::filesystem::directory_iterator{outPath}) {
		if (dir_entry.is_regular_file()) {
			++numberOfFiles;
			std::string fileName = dir_entry.path().filename().string();
			if (fileName.find("_gnss.txt") != std::string::npos) {
				gnssFilePath = dir_entry.path();
			}
		}
	}

	ASSERT_EQ(numberOfFiles, 8) << "Text logger should produce all expected files";
	ASSERT_FALSE(gnssFilePath.empty()) << "GNSS file missing";

	// GNSS file should have header + at least one data line
	std::ifstream gnssFile(gnssFilePath);
	size_t lines = 0;
	std::string line;
	while (std::getline(gnssFile, line)) {
		++lines;
	}
	ASSERT_GE(lines, 2u) << "GNSS log should contain data";
}

TEST_F(LoggerTextTestSuite, testTransferToLocalHttpsServer) {

	std::filesystem::create_directory(outPath);

	// Write cert so client (logger) trusts it
	std::string certPath = outPath + "/test_cert.pem";
	auto [cert, key] = generateSelfSignedCert();
	{
		std::ofstream certFile(certPath);
		certFile << cert;
	}
	EnvVarGuard certGuard("SSL_CERT_FILE", certPath);

	const unsigned short port = 9444;
	LocalHttpsServer server(port, cert, key);
	server.start();

	TestableLoggerText logger(outPath, ";");
	logger.setTransferConfig("127.0.0.1", std::to_string(port), "/", "test-api-key");

	// Create a dummy zip file to transfer
	std::string zipPath = outPath + "/sample_text.zip";
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

TEST_F(LoggerTextTestSuite, testTransferFailuresDoNotGrowMemory) {

	std::filesystem::create_directory(outPath);

	TestableLoggerText logger(outPath, ";");
	// Point to a closed port so connects fail fast
	logger.setTransferConfig("127.0.0.1", "65501", "/", "test-api-key");

	// Dummy zip to trigger transfer attempts
	std::string zipPath = outPath + "/sample_text_failure.zip";
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

    ros::init(argc, argv, "TestLoggerText");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
