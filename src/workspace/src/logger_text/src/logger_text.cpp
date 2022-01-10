#include "logger_text/logger_text.h"
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.hpp"
#include <cstdio>
#include <numeric>
#include <thread>


Writer::Writer(std::string & outputFolder, std::string separator):outputFolder(outputFolder),separator(separator),transformListener(buffer){
	configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
	updateSpeedThreshold();
	logger_service::GetLoggingMode::Request modeReq;
	logger_service::GetLoggingMode::Response modeRes;
	getLoggingMode(modeReq,modeRes);
	int mode = modeRes.loggingMode;
	ROS_INFO_STREAM("Logging mode set to : "<< mode <<" , "<<"Speed threshold set to : "<< speedThresholdKmh);
}

Writer::~Writer(){
	finalize();
}

void Writer::updateSpeedThreshold(){
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
	
double Writer::getSpeedThreshold(){
	return speedThresholdKmh;
}


/*
 * Creates and opens the logger files with a proper timestamp and headers into a temporary location
 */
void Writer::init(){
	//Make sure environment is sane and that logging is enabled
	if(bootstrappedGnssTime && loggerEnabled){
		fileRotationMutex.lock();

		//Make sure the files are not already opened...
		if(!gnssOutputFile && !imuOutputFile && !sonarOutputFile){

			std::string dateString = TimeUtils::getStringDate();

			//Open GNSS file
			gnssFileName = dateString + "_gnss.txt";

			std::string gnssFilePath = tmpLoggingFolder + "/"  + gnssFileName;

			gnssOutputFile= fopen(gnssFilePath.c_str(),"a");

			if(!gnssOutputFile){
				fileRotationMutex.unlock();
				throw std::invalid_argument(std::string("Couldn't open GNSS log file ") + gnssFileName);
			}

			//Open IMU file
			imuFileName = dateString + "_imu.txt";

			std::string imuFilePath = tmpLoggingFolder + "/" + imuFileName;

			imuOutputFile = fopen(imuFilePath.c_str(),"a");

        		if(!imuOutputFile){
				fileRotationMutex.unlock();
		        	throw std::invalid_argument(std::string("Couldn't open IMU log file ") + imuFileName);
        		}

			//Open sonar file
			sonarFileName = dateString + "_sonar.txt";

			std::string sonarFilePath = tmpLoggingFolder + "/" + sonarFileName;

			sonarOutputFile = fopen(sonarFilePath.c_str(),"a");

		        if(!sonarOutputFile){
				fileRotationMutex.unlock();
				throw std::invalid_argument(std::string("Couldn't open sonar log file ") + sonarFileName);
		        }

			fprintf(gnssOutputFile,"Timestamp%sLongitude%sLatitude%sEllipsoidalHeight%sStatus%sService\n",separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str());
			fprintf(imuOutputFile,"Timestamp%sHeading%sPitch%sRoll\n",separator.c_str(),separator.c_str(),separator.c_str());
			fprintf(sonarOutputFile,"Timestamp%sDepth\n",separator.c_str());
		}

		lastRotationTime = ros::Time::now();

		fileRotationMutex.unlock();
	}
}

/*
 * Closes the logging files and moves them to the web server's record directory to be accessible to download
 */

void Writer::finalize(){
	fileRotationMutex.lock();

        if(gnssOutputFile){
		//close
		fclose(gnssOutputFile);
		gnssOutputFile = NULL;

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + gnssFileName;
		std::string newPath = outputFolder + "/" + gnssFileName;
		rename(oldPath.c_str(),newPath.c_str());
	}

        if(imuOutputFile){
		//close
		fclose(imuOutputFile);
		imuOutputFile = NULL;

                //move
		std::string oldPath = tmpLoggingFolder + "/"  + imuFileName;
		std::string newPath = outputFolder + "/" + imuFileName;
		rename(oldPath.c_str(),newPath.c_str());
	}

        if(sonarOutputFile){
		//close
		fclose(sonarOutputFile);
		sonarOutputFile = NULL;

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + sonarFileName;
		std::string newPath = outputFolder + "/" + sonarFileName;
		rename(oldPath.c_str(),newPath.c_str());
	}

	fileRotationMutex.unlock();
}

/* Rotates logs based on time */
void Writer::rotate(){
	if(bootstrappedGnssTime && loggerEnabled){

		ros::Time currentTime = ros::Time::now();

		if(currentTime.toSec() - lastRotationTime.toSec() > logRotationIntervalSeconds){
			//close (if need be), then reopen files.
			finalize();
			init();
		}
	}
}

void Writer::gnssCallback(const sensor_msgs::NavSatFix& gnss){

	if(!bootstrappedGnssTime && gnss.status.status >= 0){
		bootstrappedGnssTime = true;
	}

	if(bootstrappedGnssTime && loggerEnabled){

		if(!gnssOutputFile){
			init();
		}

		uint64_t timestamp = TimeUtils::buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec);

		if(timestamp > lastGnssTimestamp){
			fprintf(gnssOutputFile,"%s%s%.10f%s%.10f%s%.3f%s%d%s%d\n",
				TimeUtils::getTimestampString(gnss.header.stamp.sec, gnss.header.stamp.nsec).c_str(),
				separator.c_str(),
				gnss.longitude,
				separator.c_str(),
				gnss.latitude,
				separator.c_str(),
				gnss.altitude,
				separator.c_str(),
				gnss.status.status,
				separator.c_str(),
				gnss.status.service
			);

			lastGnssTimestamp = timestamp;
		}
	}
}

void Writer::imuCallback(const sensor_msgs::Imu& imu){
	if(bootstrappedGnssTime && loggerEnabled){
	        double heading = 0;
        	double pitch   = 0;
	        double roll    = 0;

		uint64_t timestamp = TimeUtils::buildTimeStamp(imu.header.stamp.sec, imu.header.stamp.nsec);

		if(timestamp > lastImuTimestamp){
	                geometry_msgs::TransformStamped imuBodyTransform = buffer.lookupTransform("base_link", "imu", ros::Time(0));

	                QuaternionUtils::applyTransform(imuBodyTransform.transform.rotation,imu.orientation,heading,pitch,roll);

			heading = 90 - heading;

			//Hydrographers prefer 0-360 degree RPY
	                if(heading < 0) {
	                    heading += 360.0;
	                }

			fprintf(imuOutputFile,"%s%s%.3f%s%.3f%s%.3f\n",TimeUtils::getTimestampString(imu.header.stamp.sec, imu.header.stamp.nsec).c_str(),separator.c_str(),heading,separator.c_str(),pitch,separator.c_str(),roll);

			lastImuTimestamp = timestamp;
		}
	}
}

void Writer::speedCallback(const nav_msgs::Odometry& speed){
	ROS_DEBUG_STREAM("logger speedCallback");
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
	ROS_DEBUG_STREAM("current speed : " << current_speed);

	// wait two mins before calculating the average speed
	// TODO: this assumes 1 speed measrement per second (VTG, binary, etc). this may be false with some GNSS devices
	if (kmh_Speed_list.size() < 120){
		kmh_Speed_list.push_back(current_speed);
	}

	// Else, add the speed reading to the queue, compute the average,
	// and enable/disable logging if using a speed-based logging trigger
	else{
		kmh_Speed_list.pop_front();
		kmh_Speed_list.push_back(current_speed);
		average_speed = std::accumulate(kmh_Speed_list.begin(), kmh_Speed_list.end(), 0) / 120.0;
		logger_service::GetLoggingStatus::Request request;
		logger_service::GetLoggingStatus::Response response;

		bool isLogging = getLoggingStatus(request,response);
		if ( mode == 3 && (average_speed > speedThresholdKmh && response.status == false) ){
			//ROS_INFO("speed threshold reached, enabling logging");
			logger_service::ToggleLogging::Request toggleRequest;
			toggleRequest.loggingEnabled = true;
			logger_service::ToggleLogging::Response toggleResponse;
			toggleLogging(toggleRequest, toggleResponse);
		}
		else if(mode == 3 && (average_speed < speedThresholdKmh && response.status == true)){
			//ROS_INFO("speed below threshold, disabling logging");
			logger_service::ToggleLogging::Request toggleRequest;
			toggleRequest.loggingEnabled = false;
			logger_service::ToggleLogging::Response toggleResponse;
			toggleLogging(toggleRequest, toggleResponse);
		}
	}

}

void Writer::sonarCallback(const geometry_msgs::PointStamped& sonar){
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(sonar.header.stamp.sec, sonar.header.stamp.nsec);

		if(timestamp > lastSonarTimestamp){
			fprintf(sonarOutputFile,"%s%s%.3f\n",TimeUtils::getTimestampString(sonar.header.stamp.sec, sonar.header.stamp.nsec).c_str(),separator.c_str(),sonar.point.z);
			lastSonarTimestamp = timestamp;
		}
	}
}

bool Writer::getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response){
	response.status = this->bootstrappedGnssTime && this->loggerEnabled;
	return true;
}


bool Writer::toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response){

	if(bootstrappedGnssTime){
		mtx.lock();
		if(!loggerEnabled && request.loggingEnabled){	
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
		//ROS_WARN_STREAM("unlocking thread_id: "<<thread_id);
		return true;
	}
	ROS_WARN("Cannot toggle logger because no gpsfix");
	return false;
}

void Writer::configurationCallBack(const setting_msg::Setting &setting){
	//ROS_INFO_STREAM("logger_text configCallback -> " << setting.key << " : "<<setting.value<<"\n");
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
	        		ROS_ERROR("logging mode should be an integer 1-2-3 \n error catch in : Writer::configurationCallBack()");
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
				//ROS_INFO_STREAM(toggleResponse.loggingStatus << "  Writer::configurationCallBack() \n");

			}
		}
		else{
			ROS_ERROR_STREAM("loggingModeCallBack error"<< setting.key << " is different than 1,2,3 \n"<<"defaulting to a : always ON");
			mtx.lock();
			logger_service::SetLoggingMode defaultMode;
			defaultMode.request.loggingMode = 1;
			setLoggingMode(defaultMode.request, defaultMode.response);
			mtx.unlock();
		}
	}
	else{

	}
}

bool Writer::getLoggingMode(logger_service::GetLoggingMode::Request & req,logger_service::GetLoggingMode::Response & response){
	response.loggingMode = this->loggingMode;
	return true;
}

bool Writer::setLoggingMode(logger_service::SetLoggingMode::Request & req,logger_service::SetLoggingMode::Response & response){
	loggingMode = req.loggingMode;
	return true;
}
