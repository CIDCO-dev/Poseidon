#include "loggerBinary.h"

LoggerBinary::LoggerBinary(std::string & logFolder): LoggerBase(logFolder) {

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
		if(!outputFile.is_open()){

			outputFileName = TimeUtils::getStringDate() + std::string(".log");

            outputFile.open(outputFolder + "/" + outputFileName,std::ios::binary|std::ios::trunc);
			
			ROS_INFO("Logging binary data to %s", outputFolder.c_str());
			
            if(!outputFile.good()){
				throw std::invalid_argument("Couldn't open binary log file");
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

	if(outputFile.is_open()){
		//close
		outputFile.close();

		//move
		std::string oldPath = tmpLoggingFolder + "/"  + outputFileName;
		std::string newPath = outputFolder + "/" + outputFileName;
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

void LoggerBinary::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	if(!bootstrappedGnssTime && gnss.status.status >= 0){
		bootstrappedGnssTime = true;
	}

	if(bootstrappedGnssTime && loggerEnabled){

		if(!outputFile.is_open()){
			init();
		}
		else{
			//We will clock the log rotation check on the GNSS data since it's usually the slowest at 1Hz, which is plenty
			//TODO: we could throttle this if CPU usage becomes an issue
			rotate();
		}
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

        outputFile.write((char*)&hdr,sizeof(PacketHeader));
        outputFile.write((char*)&packet,sizeof(PositionPacket));

		lastGnssTimestamp = timestamp; //XXX unused lastGnssTimestamp variable
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
			
			imuTransform(imu, roll, pitch, heading);
			
			PacketHeader hdr;
            hdr.packetType=PACKET_ATTITUDE;
            hdr.packetSize=sizeof(AttitudePacket);
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(imu.header.stamp.sec,imu.header.stamp.nsec);

            AttitudePacket packet;
            packet.heading=heading;
            packet.pitch=pitch;
            packet.roll=roll;

            outputFile.write((char*)&hdr,sizeof(PacketHeader));
            outputFile.write((char*)&packet,sizeof(AttitudePacket));
			
			lastImuTimestamp = timestamp;
		}
	}
}

void LoggerBinary::sonarCallback(const geometry_msgs::PointStamped& sonar){
	if(bootstrappedGnssTime && loggerEnabled){
		uint64_t timestamp = TimeUtils::buildTimeStamp(sonar.header.stamp.sec, sonar.header.stamp.nsec);

		if(timestamp > lastSonarTimestamp){
			PacketHeader hdr;
            hdr.packetType=PACKET_DEPTH;
            hdr.packetSize=sizeof(DepthPacket);
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(sonar.header.stamp.sec,sonar.header.stamp.nsec);

            DepthPacket packet;

            packet.depth_x=sonar.point.x;
            packet.depth_y=sonar.point.y;
            packet.depth_z=sonar.point.z;

            outputFile.write((char*)&hdr,sizeof(PacketHeader));
            outputFile.write((char*)&packet,sizeof(DepthPacket));
            
			lastSonarTimestamp = timestamp;
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
			PacketHeader hdr;
            hdr.packetType=PACKET_LIDAR;
            hdr.packetSize=sizeof(LidarPacket) * points.size();
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(lidar.header.stamp.sec,lidar.header.stamp.nsec);

            outputFile.write((char*)&hdr,sizeof(PacketHeader));
			
			for(auto const& point : points){
    			LidarPacket packet;
            	packet.laser_x = point.x;
            	packet.laser_y = point.y;
            	packet.laser_z = point.z;
            	
            	outputFile.write((char*)&packet, sizeof(LidarPacket));
    		}
		}
		lastLidarTimestamp = timestamp;
	}
}

