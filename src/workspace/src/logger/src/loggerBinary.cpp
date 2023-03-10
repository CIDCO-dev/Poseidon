#include "loggerBinary.h"

LoggerBinary::LoggerBinary(std::string & logFolder): LoggerBase(logFolder) {
	
	//streamSubscriber = node.subscribe("gnss_bin_stream", 1000, &LoggerBinary::gnssBinStreamCallback,this, ros::TransportHints().tcpNoDelay()); //, ros::TransportHints().tcpNoDelay()

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
		if(!outputFile.is_open() && !rawGnssoutputFile.is_open() ){

			outputFileName = TimeUtils::getStringDate() + std::string(".log");
			rawGnssFileName = TimeUtils::getStringDate() + std::string(".bin");

            outputFile.open(outputFolder + "/" + outputFileName,std::ios::binary|std::ios::trunc);
			rawGnssoutputFile.open(outputFolder + "/" + rawGnssFileName,std::ios::binary|std::ios::trunc);
			
			ROS_INFO("Logging binary data to %s", outputFolder.c_str());
			
            if(!outputFile.good()){
				throw std::invalid_argument("Couldn't open binary log file");
            }
            if( !rawGnssoutputFile.good()){
				throw std::invalid_argument("Couldn't open raw gnss log file");
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

	if(outputFile.is_open() && rawGnssoutputFile.is_open()){
		//close
		outputFile.close();
		rawGnssoutputFile.close();

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + outputFileName;
		std::string newPath = outputFolder + "/" + outputFileName;
		rename(oldPath.c_str(),newPath.c_str());
		
		oldPath = tmpLoggingFolder + "/"  + rawGnssFileName;
		newPath = outputFolder + "/" + rawGnssFileName;
		rename(oldPath.c_str(),newPath.c_str());
	}
	
	fileRotationMutex.unlock();
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

void compress(std::string binSensorFileName, std::string rawGnssFileName){

	size_t posLastDot = binSensorFileName.find_last_of(".");
	// Should we handle the case scenario where theres no file extension ?
	std::string zipFilename = binSensorFileName.substr(0, posLastDot) + std::string(".zip");
	
	std::string command = "zip " + zipFilename + " " + binSensorFileName + " " + rawGnssFileName;

	ROS_INFO_STREAM(command.c_str());
	
	auto p = std::system(command.c_str());
	
	ROS_INFO_STREAM(p);
}

void LoggerBinary::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	
	if(!bootstrappedGnssTime && gnss.status.status >= 0){
		//ROS_INFO_STREAM("LoggerBinary::gnssCallback -> bootstrappedGnssTime"); ok
		bootstrappedGnssTime = true;
	}
	//ROS_INFO_STREAM("logger enabled: "<< loggerEnabled);
	//ROS_INFO("LoggerBinary::gnssCallback, gnss status %d", gnss.status.status);
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
		double roll    = 0;

		uint64_t timestamp = TimeUtils::buildTimeStamp(imu.header.stamp.sec, imu.header.stamp.nsec);

		if(timestamp > lastImuTimestamp){
			
			fileLock.lock();
			
			imuTransform(imu, roll, pitch, heading);
			
			PacketHeader hdr;
            hdr.packetType=PACKET_ATTITUDE;
            hdr.packetSize=sizeof(AttitudePacket);
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(imu.header.stamp.sec,imu.header.stamp.nsec);

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
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(sonar.header.stamp.sec,sonar.header.stamp.nsec);

            DepthPacket packet;

            packet.depth_x=sonar.point.x;
            packet.depth_y=sonar.point.y;
            packet.depth_z=sonar.point.z;

            outputFile.write((char*)&hdr, sizeof(PacketHeader));
            outputFile.write((char*)&packet, sizeof(DepthPacket));
            
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
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(lidar.header.stamp.sec,lidar.header.stamp.nsec);

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

void LoggerBinary::gnssBinStreamCallback(const binary_stream_msg::Stream& stream){
	
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = stream.timeStamp;
		
		if(timestamp > lastLidarTimestamp){
		
			char arr[stream.vector_length];
			auto v = stream.stream;
			std::copy(v.begin(), v.end(), arr);
			
			rawGnssoutputFile.write((char*)arr, stream.vector_length);
		}
		
		lastLidarTimestamp = timestamp;
	}
	
	
}
