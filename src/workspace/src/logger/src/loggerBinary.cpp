#include "loggerBinary.h"

LoggerBinary::LoggerBinary(std::string & logFolder): LoggerBase(logFolder) {
	readVitalsMsgFile();
}

LoggerBinary::~LoggerBinary(){
	finalize();
}

/*
 * Creates and opens the logger files with a proper timestamp and headers into a temporary location
 */
void LoggerBinary::init(){
	//Make sure environment is sane and that logging is enabled
	if(bootstrappedGnssTime && loggerEnabled){
		fileRotationMutex.lock();

		//Make sure the files are not already opened...
		if(!outputFile.is_open() && !rawGnssOutputFile.is_open() && !rawSonarOutputFile.is_open() ){

			outputFileName = TimeUtils::getStringDate() + std::string(".log");
			rawGnssFileName = TimeUtils::getStringDate() + this->fileExtensionForGpsDatagram;
			rawSonarFileName = TimeUtils::getStringDate() + this->fileExtensionForSonarDatagram;

			outputFile.open(outputFolder + "/" + outputFileName,std::ios::binary|std::ios::trunc);
			rawGnssOutputFile.open(outputFolder + "/" + rawGnssFileName,std::ios::binary|std::ios::trunc);
			rawSonarOutputFile.open(outputFolder + "/" + rawSonarFileName,std::ios::binary|std::ios::trunc);
			
			ROS_INFO("Logging binary data to %s", outputFolder.c_str());
			
			if(!outputFile.good()){
				throw std::invalid_argument("Couldn't open binary log file");
			}
			if( !rawGnssOutputFile.good()){
				throw std::invalid_argument("Couldn't open raw gnss log file");
			}
			if( !rawSonarOutputFile.good()){
				throw std::invalid_argument("Couldn't open raw Sonar log file");
			}

			lastRotationTime = ros::Time::now();
			fileRotationMutex.unlock();
		}
	}
}

/*
 * Closes the logging files and moves them to the web server's record directory to be accessible to download
 */

void LoggerBinary::finalize(){
	fileRotationMutex.lock();
	
	std::string newBinSensorFileName;
	std::string newRawGnssFileName;
	std::string newRawSonarFileName;
	
	if(outputFile.is_open() && rawGnssOutputFile.is_open() && rawSonarOutputFile.is_open() ){
		//close
		outputFile.close();
		rawGnssOutputFile.close();
		rawSonarOutputFile.close();
		
		//move
		std::string oldPath = tmpLoggingFolder + "/"  + outputFileName;
		newBinSensorFileName = outputFolder + "/" + outputFileName;
		rename(oldPath.c_str(), newBinSensorFileName.c_str());
		
		oldPath = tmpLoggingFolder + "/"  + rawGnssFileName;
		newRawGnssFileName = outputFolder + "/" + rawGnssFileName;
		rename(oldPath.c_str(), newRawGnssFileName.c_str());
		
		oldPath = tmpLoggingFolder + "/"  + rawSonarFileName;
		newRawSonarFileName = outputFolder + "/" + rawSonarFileName;
		rename(oldPath.c_str(), newRawSonarFileName.c_str());
	}
	
	fileRotationMutex.unlock();
	
	std::string zipFilename = rawGnssFileName.substr(0, 17) + std::string(".zip");
	
	if(activatedTransfer){
		
		std::vector<std::string> files{newBinSensorFileName, newRawGnssFileName, newRawSonarFileName};
		
		bool noError = compress(zipFilename, files);
		if(noError && can_reach_server()){
			transfer();
		}
	}
}

/* Rotates logs based on time */
void LoggerBinary::rotate(){
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

void LoggerBinary::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	
	processGnssFix(gnss);
	updateGeofence(gnss);

	if(bootstrappedGnssTime && loggerEnabled){

		if(!outputFile.is_open()){
			init();
		}
		else{
			//We will clock the log rotation check on the GNSS data since it's usually the slowest at 1Hz, which is plenty
			//TODO: we could throttle this if CPU usage becomes an issue
			rotate();
		}
		fileLock.lock();
		uint64_t timestamp = TimeUtils::buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec);
		PacketHeader hdr;
		hdr.packetType=PACKET_POSITION;
		hdr.packetSize=sizeof(PositionPacket);
		hdr.packetTimestamp=timestamp;

		PositionPacket packet;

		packet.status=gnss.status.status;
		packet.service=gnss.status.service;
		packet.longitude=gnss.longitude;
		packet.latitude=gnss.latitude;
		packet.altitude=gnss.altitude;
		memcpy(&packet.covariance,&gnss.position_covariance,9*sizeof(double));
		packet.covarianceType=gnss.position_covariance_type;

		outputFile.write((char*)&hdr, sizeof(PacketHeader));
		outputFile.write((char*)&packet, sizeof(PositionPacket));

		lastGnssTimestamp = timestamp; //XXX unused lastGnssTimestamp variable
		
		fileLock.unlock();
	}
}

void LoggerBinary::imuCallback(const sensor_msgs::Imu& imu){
	if(bootstrappedGnssTime && loggerEnabled){
		if(!outputFile.is_open()){
			init();
		}
		
		double heading = 0;
		double pitch   = 0;
		double roll	= 0;

		uint64_t timestamp = TimeUtils::buildTimeStamp(imu.header.stamp.sec, imu.header.stamp.nsec);

		if(timestamp > lastImuTimestamp){
			
			fileLock.lock();
			
			imuTransform(imu, roll, pitch, heading);
			
			PacketHeader hdr;
			hdr.packetType=PACKET_ATTITUDE;
			hdr.packetSize=sizeof(AttitudePacket);
			hdr.packetTimestamp=timestamp;

			AttitudePacket packet;
			packet.heading=heading;
			packet.pitch=pitch;
			packet.roll=roll;

			outputFile.write((char*)&hdr, sizeof(PacketHeader));
			outputFile.write((char*)&packet, sizeof(AttitudePacket));
			
			lastImuTimestamp = timestamp;
			fileLock.unlock();
		}
	}
}

void LoggerBinary::sonarCallback(const geometry_msgs::PointStamped& sonar){
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(sonar.header.stamp.sec, sonar.header.stamp.nsec);

		if(timestamp > lastSonarTimestamp){
			
			fileLock.lock();
			
			PacketHeader hdr;
			hdr.packetType=PACKET_DEPTH;
			hdr.packetSize=sizeof(DepthPacket);
			hdr.packetTimestamp=timestamp;

			DepthPacket packet;

			packet.depth_x=sonar.point.x;
			packet.depth_y=sonar.point.y;
			packet.depth_z=sonar.point.z;

			if (this->enableGeofence && not this->insideGeofence){
				ROS_INFO("Not logging as we are outside active geofence");
			}
			else {
				outputFile.write((char *) &hdr, sizeof(PacketHeader));
				outputFile.write((char *) &packet, sizeof(DepthPacket));
			}
			
			lastSonarTimestamp = timestamp;
			fileLock.unlock();
		}
	}
}

void LoggerBinary::lidarCallBack(const sensor_msgs::PointCloud2& lidar){
	//ROS_INFO_STREAM("lidar callback : " << lidar.data.size() << "\n" );
	
	sensor_msgs::PointCloud lidarXYZ;
	sensor_msgs::convertPointCloud2ToPointCloud(lidar, lidarXYZ);
	
	std::vector<geometry_msgs::Point32> points = lidarXYZ.points;
	
	
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(lidar.header.stamp.sec, lidar.header.stamp.nsec);
		
		if(timestamp > lastLidarTimestamp){
		
			fileLock.lock();
			
			PacketHeader hdr;
			hdr.packetType=PACKET_LIDAR;
			hdr.packetSize= sizeof(LidarPacket) * points.size();  //(uint64_t) (sizeof(LidarPacket) * points.size());
			hdr.packetTimestamp=timestamp;

			outputFile.write((char*)&hdr, sizeof(PacketHeader));
			
			for(auto const& point : points){
				LidarPacket packet;
				packet.laser_x = point.x;
				packet.laser_y = point.y;
				packet.laser_z = point.z;
				
				outputFile.write((char*)&packet, sizeof(LidarPacket));
			}
			
		}
		lastLidarTimestamp = timestamp;
		fileLock.unlock();
	}
}
void LoggerBinary::saveSpeed(const nav_msgs::Odometry& speed){
	
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(speed.header.stamp.sec, speed.header.stamp.nsec);

		if(timestamp > lastSonarTimestamp){
			
			fileLock.lock();
			
			PacketHeader hdr;
			hdr.packetType = PACKET_SPEED;
			hdr.packetSize=sizeof(SpeedPacket);
			hdr.packetTimestamp=timestamp;

			SpeedPacket packet;

			packet.speedKMH = speed.twist.twist.linear.y;

			outputFile.write((char*)&hdr, sizeof(PacketHeader));
			outputFile.write((char*)&packet, sizeof(SpeedPacket));
			
			lastSonarTimestamp = timestamp;
			fileLock.unlock();
		}
	}
}


/*
	Even if the content of the raspberry_vitals_msg::sysinfo changes 
	the writer and the reader will be retrocompatible
*/
void LoggerBinary::saveVitals(const raspberrypi_vitals_msg::sysinfo& msg){
	
	
	
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(msg.header.stamp.sec, msg.header.stamp.nsec);

		if( (timestamp - lastVitalsTimestamp) > (60 * 1000000) ){
			
			fileLock.lock();
			
			PacketHeader hdr;
			hdr.packetType=PACKET_VITALS;
			hdr.packetSize=sizeof(VitalsPacket);
			hdr.packetTimestamp = timestamp;

			VitalsPacket vitalsPacket;
			vitalsPacket.nbValues = this->vitalsValueName.size();
			
			outputFile.write((char*)&hdr, sizeof(PacketHeader));
			outputFile.write((char*) &vitalsPacket, sizeof(vitalsPacket));
			
			char *p = const_cast<char*>(reinterpret_cast<const char*>(&msg));
			p += sizeof(msg.header);
			
			
			for(auto valueName : vitalsValueName){
				
				VitalPacket vitalPacket;
				vitalPacket.valueNameSize = valueName.length();
				vitalPacket.valueName = valueName ;
				vitalPacket.value = *reinterpret_cast<double*>(p);
				
				outputFile.write((char*) &vitalPacket.valueNameSize, sizeof(vitalPacket.valueNameSize));
				outputFile.write(valueName.c_str(), valueName.length());
				outputFile.write((char*) &vitalPacket.value, sizeof(double));
				
				p += sizeof(double);
			}

			
			lastVitalsTimestamp = timestamp;
			fileLock.unlock();
		}
	}

}

void LoggerBinary::readVitalsMsgFile(){

	std::ifstream file("/opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg");
	if (!file.is_open()) {
		ROS_ERROR("Error opening file: /opt/Poseidon/src/workspace/src/raspberrypi_vitals_msg/msg/sysinfo.msg");
		return;
	}

	std::string line, firstCol, word;
	
	while (std::getline(file, line)) {
		std::istringstream ss(line);
		ss >> firstCol >> word;
		
		vitalsValueName.push_back(word);
		vitalsValueNameSize.push_back(word.length());
	}
	vitalsValueName.erase(vitalsValueName.begin());

	file.close();
}
