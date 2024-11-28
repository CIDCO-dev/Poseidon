#include "loggerText.h"

LoggerText::LoggerText(std::string & logFolder, std::string separator):LoggerBase(logFolder), separator(separator){
	readVitalsMsgFile();
}

LoggerText::~LoggerText(){
	finalize();
}

/*
 * Creates and opens the logger files with a proper timestamp and headers into a temporary location
 */
void LoggerText::init(){
	//Make sure environment is sane and that logging is enabled
	if(bootstrappedGnssTime && loggerEnabled){
		fileRotationMutex.lock();

		//Make sure the files are not already opened...
		if(!gnssOutputFile && !imuOutputFile && !sonarOutputFile && !lidarOutputFile && !rawGnssOutputFile.is_open() && !rawGnssOutputFile.is_open() && !vitalsOutputFile && !speedOutputFile){

			std::string dateString = TimeUtils::getStringDate();
			
			initGnssFile(dateString);
			initImuFile(dateString);
			initSonarFile(dateString);
			initLidarFile(dateString);
			initSpeedFile(dateString);
			initVitalsFile(dateString);
			
			rawGnssFileName = dateString + this->fileExtensionForGpsDatagram;
			rawGnssOutputFile.open(tmpLoggingFolder + "/" + rawGnssFileName,std::ios::binary|std::ios::trunc);
			
			if( !rawGnssOutputFile.good()){
				throw std::invalid_argument("Couldn't open raw gnss log file");
			}
			
			rawSonarFileName = dateString + this->fileExtensionForSonarDatagram;
			rawSonarOutputFile.open(tmpLoggingFolder + "/" + rawSonarFileName,std::ios::binary|std::ios::trunc);
			
			if( !rawSonarOutputFile.good()){
				throw std::invalid_argument("Couldn't open raw sonar log file");
			}
			
		}

		lastRotationTime = ros::Time::now();

		fileRotationMutex.unlock();
	}
}

void LoggerText::initGnssFile(std::string &dateString){
	//Open GNSS file
	gnssFileName = dateString + "_gnss.txt";

	std::string gnssFilePath = tmpLoggingFolder + "/"  + gnssFileName;

	gnssOutputFile= fopen(gnssFilePath.c_str(),"a");

	if(!gnssOutputFile){
		fileRotationMutex.unlock();
		throw std::invalid_argument(std::string("Couldn't open GNSS log file ") + gnssFileName);
	}
	
	fprintf(gnssOutputFile,"Timestamp%sLongitude%sLatitude%sEllipsoidalHeight%sStatus%sService\n",
									separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str());
}

void LoggerText::initImuFile(std::string &dateString){
//Open IMU file
	imuFileName = dateString + "_imu.txt";

	std::string imuFilePath = tmpLoggingFolder + "/" + imuFileName;

	imuOutputFile = fopen(imuFilePath.c_str(),"a");

	if(!imuOutputFile){
		fileRotationMutex.unlock();
		throw std::invalid_argument(std::string("Couldn't open IMU log file ") + imuFileName);
	}
	
	fprintf(imuOutputFile,"Timestamp%sHeading%sPitch%sRoll\n",separator.c_str(),separator.c_str(),separator.c_str());
	
}

void LoggerText::initSonarFile(std::string &dateString){
	//Open sonar file
	sonarFileName = dateString + "_sonar.txt";

	std::string sonarFilePath = tmpLoggingFolder + "/" + sonarFileName;

	sonarOutputFile = fopen(sonarFilePath.c_str(),"a");

	if(!sonarOutputFile){
		fileRotationMutex.unlock();
		throw std::invalid_argument(std::string("Couldn't open sonar log file ") + sonarFileName);
	}
	
	fprintf(sonarOutputFile,"Timestamp%sDepth\n",separator.c_str());
}

void LoggerText::initLidarFile(std::string &dateString){
	//Open lidar file
	lidarFileName = dateString + "_lidar.txt";

	std::string lidarFilePath = tmpLoggingFolder + "/" + lidarFileName;

	lidarOutputFile = fopen(lidarFilePath.c_str(),"a");

	if(!lidarOutputFile){
		fileRotationMutex.unlock();
		throw std::invalid_argument(std::string("Couldn't open lidar log file ") + lidarFileName);
	}
	
	fprintf(lidarOutputFile,"Timestamp%sPoints\n",separator.c_str());
}

void LoggerText::initSpeedFile(std::string &dateString){
	//Open speed file
	speedFilename = dateString + "_speed.txt";

	std::string speedFilePath = tmpLoggingFolder + "/" + speedFilename;

	speedOutputFile = fopen(speedFilePath.c_str(),"a");

	if(!speedOutputFile){
		fileRotationMutex.unlock();
		throw std::invalid_argument(std::string("Couldn't open speed log file ") + speedFilename);
	}
	
	fprintf(speedOutputFile,"Timestamp%sSpeed\n",separator.c_str());
}

void LoggerText::initVitalsFile(std::string &dateString){
	//Open vitals file
	vitalsFilename = dateString + "_vitals.txt";

	std::string vitalsFilePath = tmpLoggingFolder + "/" + vitalsFilename;

	vitalsOutputFile = fopen(vitalsFilePath.c_str(),"a");


	if(!vitalsOutputFile){
		fileRotationMutex.unlock();
		throw std::invalid_argument(std::string("Couldn't open vital log file ") + vitalsFilename);
	}
	
	fprintf(vitalsOutputFile,"Timestamp%sCPU Temp%sCPU Used%sFREE RAM%sFREE HDD%sUptime%sHumidity%sSystem Temp%sSupply Voltage\n",
									separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str());

}


/*
 * Closes the logging files and moves them to the web server's record directory to be accessible to download
 */

void LoggerText::finalize(){
	fileRotationMutex.lock();

	std::string newGnssPath;
	std::string newImuPath;
	std::string newSonarPath;
	std::string newLidarPath;
	std::string newRawGnssPath;
	std::string newRawSonarPath;
	std::string newSpeedPath;
	std::string newVitalsPath;
	
	if(gnssOutputFile){
		//close
		fclose(gnssOutputFile);
		gnssOutputFile = NULL;

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + gnssFileName;
		newGnssPath = outputFolder + "/" + gnssFileName;
		rename(oldPath.c_str(), newGnssPath.c_str());
	}

	if(imuOutputFile){
		//close
		fclose(imuOutputFile);
		imuOutputFile = NULL;

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + imuFileName;
		newImuPath = outputFolder + "/" + imuFileName;
		rename(oldPath.c_str(), newImuPath.c_str());
	}

	if(sonarOutputFile){
		//close
		fclose(sonarOutputFile);
		sonarOutputFile = NULL;

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + sonarFileName;
		newSonarPath = outputFolder + "/" + sonarFileName;
		rename(oldPath.c_str(), newSonarPath.c_str());
	}
	
	if(lidarOutputFile){
		//close
		fclose(lidarOutputFile);
		lidarOutputFile = NULL;

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + lidarFileName;
		newLidarPath = outputFolder + "/" + lidarFileName;
		rename(oldPath.c_str(), newLidarPath.c_str());
	}
	
	if(rawGnssOutputFile.is_open()){
		//close
		rawGnssOutputFile.close();
		
		std::string oldPath = tmpLoggingFolder + "/"  + rawGnssFileName;
		newRawGnssPath = outputFolder + "/" + rawGnssFileName;
		rename(oldPath.c_str(), newRawGnssPath.c_str());
	}
	
	if(rawSonarOutputFile.is_open()){
		//close
		rawSonarOutputFile.close();
		
		std::string oldPath = tmpLoggingFolder + "/"  + rawSonarFileName;
		newRawSonarPath = outputFolder + "/" + rawSonarFileName;
		rename(oldPath.c_str(), newRawSonarPath.c_str());
	}
	
	if(speedOutputFile){
		//close
		fclose(speedOutputFile);
		speedOutputFile = NULL;

		
		std::string oldPath = tmpLoggingFolder + "/"  + speedFilename;
		newSpeedPath = outputFolder + "/" + speedFilename;
		rename(oldPath.c_str(), newSpeedPath.c_str());
	}
	
	if(vitalsOutputFile){
		//close
		fclose(vitalsOutputFile);
		vitalsOutputFile = NULL;
		
		std::string oldPath = tmpLoggingFolder + "/"  + vitalsFilename;
		newVitalsPath = outputFolder + "/" + vitalsFilename;
		rename(oldPath.c_str(), newVitalsPath.c_str());
	}
	
	fileRotationMutex.unlock();

	std::string zipFilename = gnssFileName.substr(0, 17) + std::string(".zip");
	
	if(activatedTransfer){
	
		std::vector<std::string> files{newGnssPath, newImuPath, newSonarPath, newLidarPath, newRawGnssPath, newRawSonarPath, newSpeedPath, newVitalsPath};
	
		bool noError = compress(zipFilename, files);
		if(noError && can_reach_server()){
			transfer();
		}
	}
}

/* Rotates logs based on time */
void LoggerText::rotate(){
	if(bootstrappedGnssTime && loggerEnabled){

		ros::Time currentTime = ros::Time::now();

		if(currentTime.toSec() - lastRotationTime.toSec() > logRotationIntervalSeconds){
			ROS_INFO("Rotating logs");
			//close (if need be), then reopen files.
			finalize();
			init();
		}
	}
}


void LoggerText::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	
	processGnssFix(gnss);
	updateGeofence(gnss);
	if(!bootstrappedGnssTime && gnss.status.status >= 0){
		bootstrappedGnssTime = true;
	}

	if(bootstrappedGnssTime && loggerEnabled){

		if(!gnssOutputFile){
			init();
		}
		else{
			//We will clock the log rotation check on the GNSS data since it's usually the slowest at 1Hz, which is plenty
			//TODO: we could throttle this if CPU usage becomes an issue
			rotate();
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

void LoggerText::imuCallback(const sensor_msgs::Imu& imu){
	if(bootstrappedGnssTime && loggerEnabled){
		double heading = 0;
		double pitch   = 0;
		double roll	= 0;

		uint64_t timestamp = TimeUtils::buildTimeStamp(imu.header.stamp.sec, imu.header.stamp.nsec);

		if(timestamp > lastImuTimestamp){
			
			imuTransform(imu, roll, pitch, heading);
			
			fprintf(imuOutputFile,"%s%s%.3f%s%.3f%s%.3f\n",TimeUtils::getTimestampString(imu.header.stamp.sec, imu.header.stamp.nsec).c_str(),separator.c_str(),heading,separator.c_str(),pitch,separator.c_str(),roll);

			lastImuTimestamp = timestamp;
		}
	}
}

void LoggerText::sonarCallback(const geometry_msgs::PointStamped& sonar){
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(sonar.header.stamp.sec, sonar.header.stamp.nsec);

		if(timestamp > lastSonarTimestamp){
			/*fprintf(sonarOutputFile,"%s%s%.3f\n",TimeUtils::getTimestampString(sonar.header.stamp.sec, sonar.header.stamp.nsec).c_str(),separator.c_str(),sonar.point.z);*/
			if (this->enableGeofence && not this->insideGeofence){
				ROS_INFO("Not logging as we are outside active geofence");
			}
			else {
				fprintf(sonarOutputFile,"%s%s%.3f\n",TimeUtils::getTimestampString(sonar.header.stamp.sec, sonar.header.stamp.nsec).c_str(),separator.c_str(),sonar.point.z);
			}
			lastSonarTimestamp = timestamp;
		}
	}
}

void LoggerText::lidarCallBack(const sensor_msgs::PointCloud2& lidar){
	//ROS_INFO_STREAM("lidar callback : " << lidar.data.size() << "\n" );
	
	sensor_msgs::PointCloud lidarXYZ;
	sensor_msgs::convertPointCloud2ToPointCloud(lidar, lidarXYZ);
	
	std::vector<geometry_msgs::Point32> points = lidarXYZ.points;
	
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(lidar.header.stamp.sec, lidar.header.stamp.nsec);
		
		if(timestamp > lastLidarTimestamp){
			fprintf(lidarOutputFile,"%s%s",TimeUtils::getTimestampString(lidar.header.stamp.sec, lidar.header.stamp.nsec).c_str(), separator.c_str());
			lastLidarTimestamp = timestamp;
			for(auto const& point : points){
				fprintf(lidarOutputFile,"%f %f %f%s", point.x, point.y, point.z, separator.c_str());
			}
			fprintf(lidarOutputFile,"\n");
		}
	}
}

void LoggerText::saveSpeed(const nav_msgs::Odometry& speed){
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(speed.header.stamp.sec, speed.header.stamp.nsec);

		if(timestamp > lastSonarTimestamp){
			fprintf(speedOutputFile,"%s%s%.3f\n",TimeUtils::getTimestampString(speed.header.stamp.sec, speed.header.stamp.nsec).c_str(),separator.c_str(),speed.twist.twist.linear.y);
			lastSonarTimestamp = timestamp;
		}
	}
}

void LoggerText::saveVitals(const raspberrypi_vitals_msg::sysinfo& msg){
	
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(msg.header.stamp.sec, msg.header.stamp.nsec);

		if( (timestamp - lastVitalsTimestamp) > (60 * 1000000) ){
			
			fprintf(vitalsOutputFile,"%s%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f%s%.3f\n",
				TimeUtils::getTimestampString(msg.header.stamp.sec, msg.header.stamp.nsec).c_str(),
				separator.c_str(),
				msg.cputemp,
				separator.c_str(),
				msg.cpuload,
				separator.c_str(),
				msg.freeram,
				separator.c_str(),
				msg.freehdd,
				separator.c_str(),
				msg.uptime,
				separator.c_str(),
				msg.humidity,
				separator.c_str(),
				msg.temperature,
				separator.c_str(),
				msg.voltage
				);
			
			lastVitalsTimestamp = timestamp;
		}
	}
	
}

void LoggerText::readVitalsMsgFile(){

	std::ifstream file("/opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg");
	if (!file.is_open()) {
		ROS_ERROR("Error opening file: /opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg");
		return;
	}

	std::string line, firstCol, word;
	
	while (std::getline(file, line)) {
		std::istringstream ss(line);
		ss >> firstCol >> word;
		
		this->vitalsHeader.push_back(word);
	}
	this->vitalsHeader[0] = "timeStamp";

	file.close();
}
