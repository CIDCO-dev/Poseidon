#include "logger_text/logger_text.h"
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.hpp"
#include <cstdio>

Writer::Writer(std::string & outputFolder, std::string separator):outputFolder(outputFolder),separator(separator),transformListener(buffer){

}

Writer::~Writer(){
	finalize();
}

void Writer::init(){
	std::string dateString = TimeUtils::getStringDate();

	//Open GNSS file
	std::string gnssFileName = outputFolder + "/"  + dateString + "_gnss.txt";

	gnssOutputFile= fopen(gnssFileName.c_str(),"a");

	if(!gnssOutputFile){
		throw std::invalid_argument(std::string("Couldn't open GNSS log file ") + gnssFileName);
	}

	//Open IMU file
	std::string imuFileName = outputFolder + "/" + dateString + "_imu.txt";

	imuOutputFile = fopen(imuFileName.c_str(),"a");

        if(!imuOutputFile){
	        throw std::invalid_argument(std::string("Couldn't open IMU log file ") + imuFileName);
        }

	//Open sonar file
	std::string sonarFileName = outputFolder + "/" + dateString + "_sonar.txt";

	sonarOutputFile = fopen(sonarFileName.c_str(),"a");

        if(!sonarOutputFile){
 	       throw std::invalid_argument(std::string("Couldn't open sonar log file ") + sonarFileName);
        }

	fprintf(gnssOutputFile,"Timestamp%sLongitude%sLatitude%sEllipsoidalHeight%sStatus%sService\n",separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str(),separator.c_str());

	fprintf(imuOutputFile,"Timestamp%sHeading%sPitch%sRoll\n",separator.c_str(),separator.c_str(),separator.c_str());

	fprintf(sonarOutputFile,"Timestamp%sDepth\n",separator.c_str());


}

void Writer::finalize(){
        if(gnssOutputFile)   fclose(gnssOutputFile);
        if(imuOutputFile)    fclose(imuOutputFile);
        if(sonarOutputFile)  fclose(sonarOutputFile);

	gnssOutputFile  = NULL;
	imuOutputFile   = NULL;
	sonarOutputFile = NULL;
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

void Writer::imuCallback(const nav_msgs::Odometry& odom){
	if(bootstrappedGnssTime && loggerEnabled){
	        double heading = 0;
        	double pitch   = 0;
	        double roll    = 0;

		uint64_t timestamp = TimeUtils::buildTimeStamp(odom.header.stamp.sec, odom.header.stamp.nsec);

		if(timestamp > lastImuTimestamp){
	                geometry_msgs::TransformStamped imuBodyTransform = buffer.lookupTransform("base_link", "imu", ros::Time(0));

	                QuaternionUtils::applyTransform(imuBodyTransform.transform.rotation,odom.pose.pose.orientation,heading,pitch,roll);

			if(heading < 0){
				heading += 360.0;
			}

			fprintf(imuOutputFile,"%s%s%.3f%s%.3f%s%.3f\n",TimeUtils::getTimestampString(odom.header.stamp.sec, odom.header.stamp.nsec).c_str(),separator.c_str(),heading,separator.c_str(),pitch,separator.c_str(),roll);

			lastImuTimestamp = timestamp;
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
	response.status = bootstrappedGnssTime && loggerEnabled;
	return true;
}


bool Writer::toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response){
	mtx.lock();

	if(bootstrappedGnssTime){
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

		mtx.unlock();

		response.loggingStatus=loggerEnabled;
		return true;
	}

	return false;
}
