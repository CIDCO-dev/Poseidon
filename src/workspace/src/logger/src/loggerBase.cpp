#include "loggerBase.h"

LoggerBase::LoggerBase(std::string & outputFolder):outputFolder(outputFolder), transformListener(buffer){
	
	gnssSubscriber = node.subscribe("fix", 1000, &LoggerBase::gnssCallback, this);
	imuSubscriber = node.subscribe("imu/data", 1000, &LoggerBase::imuCallback, this);
	depthSubscriber = node.subscribe("depth", 1000, &LoggerBase::sonarCallback, this);
	speedSubscriber = node.subscribe("speed", 1000, &LoggerBase::speedCallback, this);
	configurationSubscriber = node.subscribe("configuration", 1000, &LoggerBase::configurationCallBack, this);
	lidarSubscriber = node.subscribe("velodyne_points", 1000, &LoggerBase::lidarCallBack, this);
	streamSubscriber = node.subscribe("gnss_bin_stream", 1000, &LoggerBase::gnssBinStreamCallback, this, ros::TransportHints().tcpNoDelay());
	hddVitalsSubscriber = node.subscribe("vitals", 1000, &LoggerBase::hddVitalsCallback, this);
	
	getLoggingStatusService = node.advertiseService("get_logging_status", &LoggerBase::getLoggingStatus, this);
	toggleLoggingService = node.advertiseService("toggle_logging", &LoggerBase::toggleLogging, this);
	
	getLoggingModeService = node.advertiseService("get_logging_mode", &LoggerBase::getLoggingMode, this);
	setLoggingModeService = node.advertiseService("set_logging_mode", &LoggerBase::setLoggingMode, this);
	
	configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
	updateSpeedThreshold();
	updateLoggingMode();
	ROS_INFO_STREAM("Logging mode set to : "<< loggingMode <<" , "<<"Speed threshold set to : "<< speedThresholdKmh);
}

LoggerBase::~LoggerBase(){
}


void LoggerBase::updateLoggingMode(){
	setting_msg::ConfigurationService srv;

    srv.request.key = "loggingMode";
    if(configurationClient.call(srv)){
    	std::string strLoggingMode = srv.response.value;
        try{
        	loggingMode = stod(strLoggingMode);
        	if(loggingMode != 1 && loggingMode != 2 && loggingMode != 3){
        		ROS_ERROR("Invalid logging mode, defaulting to Always On (1) \nValid modes are:\n Always On (1) \n Manual (2) \n Speed-Based (3)");
        		loggingMode = 1;
        	}
        }
        catch(std::invalid_argument &err){
        	ROS_INFO_STREAM("logging mode from config file is not written properly \n example : 1");
        }
    }
    else{
    	ROS_WARN("no logging mode define in config file, defaulting to Always on");
        loggingMode = 1;
    }
    
}

void LoggerBase::updateSpeedThreshold(){
	setting_msg::ConfigurationService srv;

    srv.request.key = "speedThresholdKmh";

    if(configurationClient.call(srv)){
    	std::string speedThresKmh = srv.response.value;
    	//ROS_INFO_STREAM("speed threshold from config file : " << speedThresKmh);
        try{
        	speedThresholdKmh = stod(speedThresKmh);
        }
        catch(std::invalid_argument &err){
        	ROS_INFO_STREAM("speed threshold from config file is not written properly \n example : 4.4");
        }
    }
    else{
    	ROS_WARN("no speed threshold define in config file, defaulting to 5 Kmh");
        speedThresholdKmh = 5.0;
    }
}

double LoggerBase::getSpeedThreshold(){
	return speedThresholdKmh;
}

bool LoggerBase::getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response){
	response.status = this->bootstrappedGnssTime && this->loggerEnabled;
	return true;
}


bool LoggerBase::toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response){

	if(bootstrappedGnssTime){
		mtx.lock();
		if(!loggerEnabled && request.loggingEnabled && hddFreeSpaceOK){	
			//Enabling logging, init logfiles
			loggerEnabled=true;
			init();
		}
		else if(loggerEnabled && !request.loggingEnabled){
			//shutting down logging, finalize logfiles
			loggerEnabled=false;
			finalize();
		}

		response.loggingStatus=loggerEnabled;
		mtx.unlock();
		return true;
	}
	else{
		ROS_WARN("Cannot toggle logger because no gpsfix");
		return false;
	}
}

void LoggerBase::configurationCallBack(const setting_msg::Setting &setting){
	// ROS_INFO_STREAM("logger_text configCallback -> " << setting.key << " : "<<setting.value<<"\n");
	if(setting.key == "loggingMode"){
		if(setting.value == "1" || setting.value == "2" || setting.value == "3"){
			try{
				mtx.lock();
				int mode = stoi(setting.value);
				logger_service::SetLoggingMode newMode;
				newMode.request.loggingMode = mode;
				setLoggingMode(newMode.request, newMode.response);
				mtx.unlock();
			}
			catch(std::invalid_argument &err){
				mtx.lock();
	        		ROS_ERROR("logging mode should be an integer 1-2-3 \n error catch in : LoggerBase::configurationCallBack()");
        			logger_service::SetLoggingMode defaultMode;
				defaultMode.request.loggingMode = 1;
				setLoggingMode(defaultMode.request, defaultMode.response);
				mtx.unlock();
        		}

			logger_service::GetLoggingStatus::Request request;
			logger_service::GetLoggingStatus::Response response;
			bool isLogging = getLoggingStatus(request,response);

			if (setting.value == "1" && response.status != true){ 
				ROS_INFO("Logging set to always ON");
				logger_service::ToggleLogging::Request toggleRequest;
				toggleRequest.loggingEnabled = true;
				logger_service::ToggleLogging::Response toggleResponse;
				toggleLogging(toggleRequest, toggleResponse);
				//ROS_INFO_STREAM(toggleResponse.loggingStatus << "  LoggerBase::configurationCallBack() \n");

			}
		}
		else{
			ROS_ERROR_STREAM("loggingModeCallBack error "<< setting.key << " is different than 1,2,3 \n"<<"defaulting to: always ON");
			mtx.lock();
			logger_service::SetLoggingMode defaultMode;
			defaultMode.request.loggingMode = 1;
			setLoggingMode(defaultMode.request, defaultMode.response);
			mtx.unlock();
		}
	}
	else if(setting.key == "targetServer"){
		if(setting.value == ""){
			this->activatedTransfer = false;
		}
		else{
			this->host = setting.value;
			this->activatedTransfer = true;
		}
	}
	else if(setting.key == "apiTarget"){
		if(setting.value == ""){
			this->target = "/";
		}
		else{
			this->target = setting.value;
		}
	}
	else if(setting.key == "logRotationIntervalSeconds"){
		ROS_INFO_STREAM("configurationCallBack(logRotationIntervalSeconds): " << setting.value);
		if(setting.value == ""){
			this->logRotationIntervalSeconds = 3600;
			ROS_INFO_STREAM("Log rotation interval from config file is not set \n Defaulting to 1h");
		}
		else{
			try{
        		this->logRotationIntervalSeconds = stoi(setting.value);
        	}
        	catch(std::invalid_argument &err){
        		ROS_ERROR_STREAM("Log rotation interval from config file is not set properly \n example : 3600 \n Defaulting to 1h");
        		this->logRotationIntervalSeconds = 3600;
        	}
		}
	}
}

bool LoggerBase::getLoggingMode(logger_service::GetLoggingMode::Request & req,logger_service::GetLoggingMode::Response & response){
	response.loggingMode = this->loggingMode;
	return true;
}

bool LoggerBase::setLoggingMode(logger_service::SetLoggingMode::Request & req,logger_service::SetLoggingMode::Response & response){
	loggingMode = req.loggingMode;
	return true;
}

void LoggerBase::speedCallback(const nav_msgs::Odometry& speed){
	logger_service::GetLoggingMode::Request modeReq;
	logger_service::GetLoggingMode::Response modeRes;
	getLoggingMode(modeReq,modeRes);
	int mode = modeRes.loggingMode;

	speedThresholdKmh = getSpeedThreshold();

	if(speedThresholdKmh < 0 && speedThresholdKmh > 100){
		speedThresholdKmh = defaultSpeedThreshold;
		std::string speed = std::to_string(speedThresholdKmh);
		ROS_ERROR_STREAM("invalid speed threshold, defaulting to "<<speed<<" Kmh");
	}

	double current_speed = speed.twist.twist.linear.y;
	//ROS_DEBUG_STREAM("current speed : " << current_speed);

	// wait two mins before calculating the average speed
	// TODO: this assumes 1 speed measrement per second (VTG, binary, etc). this may be false with some GNSS devices
	if (kmhSpeedList.size() < 120){
		kmhSpeedList.push_back(current_speed);
	}

	// Else, add the speed reading to the queue, compute the average,
	// and enable/disable logging if using a speed-based logging trigger
	else{
		kmhSpeedList.pop_front();
		kmhSpeedList.push_back(current_speed);
		averageSpeed = std::accumulate(kmhSpeedList.begin(), kmhSpeedList.end(), 0) / 120.0;
		logger_service::GetLoggingStatus::Request request;
		logger_service::GetLoggingStatus::Response response;

		bool isLogging = getLoggingStatus(request,response);
		if ( mode == 3 && (averageSpeed > speedThresholdKmh && response.status == false) ){
			//ROS_INFO("speed threshold reached, enabling logging");
			logger_service::ToggleLogging::Request toggleRequest;
			toggleRequest.loggingEnabled = true;
			logger_service::ToggleLogging::Response toggleResponse;
			toggleLogging(toggleRequest, toggleResponse);
		}
		else if(mode == 3 && (averageSpeed < speedThresholdKmh && response.status == true)){
			//ROS_INFO("speed below threshold, disabling logging");
			logger_service::ToggleLogging::Request toggleRequest;
			toggleRequest.loggingEnabled = false;
			logger_service::ToggleLogging::Response toggleResponse;
			toggleLogging(toggleRequest, toggleResponse);
		}
	}

}

void LoggerBase::imuTransform(const sensor_msgs::Imu& imu, double & roll , double & pitch, double & heading){
	
	geometry_msgs::TransformStamped imuBodyTransform = buffer.lookupTransform("base_link", "imu", ros::Time(0));

	QuaternionUtils::applyTransform(imuBodyTransform.transform.rotation,imu.orientation,heading,pitch,roll);

	heading = 90 - heading; //XXX

	//Hydrographers prefer 0-360 degree RPY
	if(heading < 0) {
		heading += 360.0;
	}
	
}

void LoggerBase::transfer(){
	
	std::filesystem::path outputFolderPath = outputFolder;
	for(auto &dir_entry: std::filesystem::directory_iterator{outputFolderPath}){
		if(dir_entry.is_regular_file() && dir_entry.path().extension() == ".zip"){
		    	    
	    	std::string base64Zip = zip_to_base64(dir_entry.path());
	    	std::string json = create_json_str(base64Zip);
	    	bool ok = send_job(json);
	    	
	    	if(!ok){
	    		// XXX retry  3-5 time ??
	    		break;
	    	}
	    	else{
	    		if(!std::filesystem::remove(dir_entry.path())){
	    			ROS_ERROR_STREAM("Could not delete file:" << dir_entry.path());
	    		}
	    	}
	    }
	}	
}

std::string LoggerBase::zip_to_base64(std::string zipPath){

	typedef boost::archive::iterators::base64_from_binary<boost::archive::iterators::transform_width<std::string::const_iterator,6,8> > it_base64_t;
	std::string s;
	
	std::ifstream file(zipPath, std::ios::in | std::ios::binary);

	if (file.is_open())
	{
		s.append(std::string(std::istreambuf_iterator<char>(file), {}));
		file.close();
	}
	else{
		throw std::ios_base::failure("Failed to open the file");
	}

  // Encode
  unsigned int writePaddChars = (3-s.length()%3)%3;
  std::string base64(it_base64_t(s.begin()),it_base64_t(s.end()));
  base64.append(writePaddChars,'=');

  return base64;
}

std::string LoggerBase::create_json_str(std::string &base64Zip){
	
	rapidjson::Document d;
	d.SetObject();

	rapidjson::Value key("classified");
	d.AddMember("apiKey", key, d.GetAllocator());
	
	rapidjson::Value jobType("captain crunch");
	d.AddMember("jobType", jobType, d.GetAllocator());
	
	rapidjson::Value fileData;
	char* buffer = new char [base64Zip.size()+1];
	int len = sprintf(buffer, "%s", base64Zip.c_str());
	fileData.SetString(buffer, len, d.GetAllocator());
	delete buffer;
	d.AddMember("fileData", fileData, d.GetAllocator());
	
	rapidjson::StringBuffer sb;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
	d.Accept(writer);
	return sb.GetString();
}

bool LoggerBase::send_job(std::string json){
	bool status;
	
	try{
		int version = 11;
		
        // The io_context is required for all I/O
        boost::asio::io_context ioService;

        // These objects perform our I/O
        boost::asio::ip::tcp::resolver resolver(ioService);
        boost::beast::tcp_stream stream(ioService);

        // Look up the domain name
        auto const results = resolver.resolve(this->host, "5000");

        // Make the connection on the IP address we get from a lookup
        stream.connect(results);

        // Set up an HTTP GET request message
        boost::beast::http::request<boost::beast::http::string_body> req{boost::beast::http::verb::post, this->target, version};
        req.set(boost::beast::http::field::host, this->host);
        req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
		
		req.body() = json;
		req.prepare_payload();
		
        // Send the HTTP request to the remote host
        boost::beast::http::write(stream, req);

        // This buffer is used for reading and must be persisted
        boost::beast::flat_buffer buffer;

        // Declare a container to hold the response
        boost::beast::http::response<boost::beast::http::dynamic_body> res;

        // Receive the HTTP response
        boost::beast::http::read(stream, buffer, res);
        boost::beast::error_code ec;

		if(res.result() == boost::beast::http::status::ok){
			stream.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
		    if(ec && ec != boost::beast::errc::not_connected){
		        throw boost::beast::system_error{ec};
		    }
		    status = true;
        }
        else{
         	stream.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        	if(ec && ec != boost::beast::errc::not_connected){
            	throw boost::beast::system_error{ec};
            }
            status = false;
        }

    }
    catch(std::exception const& e)
    {
        ROS_ERROR_STREAM("Post request error: " << e.what());
    }
    return status;
}

bool LoggerBase::can_reach_server(){
	bool status;
	try{
		int version = 11;
		
        // The io_context is required for all I/O
        boost::asio::io_context ioService;

        // These objects perform our I/O
        boost::asio::ip::tcp::resolver resolver(ioService);
        boost::beast::tcp_stream stream(ioService);

        // Look up the domain name
        auto const results = resolver.resolve(this->host, "5000");

        // Make the connection on the IP address we get from a lookup
        stream.connect(results);

        // Set up an HTTP GET request message
        boost::beast::http::request<boost::beast::http::string_body> req{boost::beast::http::verb::get, this->target, version};
        req.set(boost::beast::http::field::host, this->host);
        req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
		
        // Send the HTTP request to the remote host
        boost::beast::http::write(stream, req);

        // This buffer is used for reading and must be persisted
        boost::beast::flat_buffer buffer;

        // Declare a container to hold the response
        boost::beast::http::response<boost::beast::http::dynamic_body> res;

        // Receive the HTTP response
        boost::beast::http::read(stream, buffer, res);
        boost::beast::error_code ec;

		if(res.result() == boost::beast::http::status::ok){
			stream.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
		    if(ec && ec != boost::beast::errc::not_connected){
		        throw boost::beast::system_error{ec};
		    }
		    status = true;
        }
        else{
         	stream.socket().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        	if(ec && ec != boost::beast::errc::not_connected){
            	throw boost::beast::system_error{ec};
            }
            status = false;
        }

    }
    catch(std::exception const& e)
    {
        ROS_ERROR_STREAM("Get request error: " << e.what());
    }
    
    return status;
}


void LoggerBase::hddVitalsCallback(const raspberrypi_vitals_msg::sysinfo vitals){
	
	if(vitals.freehdd < 10.0 ){
		
		this->hddFreeSpaceOK = false;
		
		logger_service::ToggleLogging::Request toggleRequest;
		toggleRequest.loggingEnabled = false;
		logger_service::ToggleLogging::Response toggleResponse;
		toggleLogging(toggleRequest, toggleResponse);
				
	}
	
	if(vitals.freehdd > 11.0 ){
		this->hddFreeSpaceOK = true;
	}
	
}


