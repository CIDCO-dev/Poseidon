#include "logger_text/logger_text.h"
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"
#include <cstdio>

Writer::Writer(std::string & outputFolder, std::string separator):outputFolder(outputFolder),separator(separator){
	if(loggerEnabled){
		init();
	}
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

	fprintf(gnssOutputFile,"Timestamp%sLongitude%sLatitude%sEllipsoidalHeight\n",separator.c_str(),separator.c_str(),separator.c_str());

	fprintf(imuOutputFile,"Timestamp%sHeading%sPitch%sRoll\n",separator.c_str(),separator.c_str(),separator.c_str());

	fprintf(sonarOutputFile,"Timestamp%sDepth\n",separator.c_str());


}

void Writer::finalize(){
        if(gnssOutputFile)   fclose(gnssOutputFile);
        if(imuOutputFile)    fclose(imuOutputFile);
        if(sonarOutputFile)  fclose(sonarOutputFile);
}

void Writer::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	if(loggerEnabled){
		fprintf(gnssOutputFile,"%s%s%.8f%s%.8f%s%.3f\n",TimeUtils::getTimestampString(gnss.header.stamp.sec, gnss.header.stamp.nsec).c_str(),separator.c_str(),gnss.longitude,separator.c_str(),gnss.latitude,separator.c_str(),gnss.altitude);
	}
}

void Writer::imuCallback(const nav_msgs::Odometry& odom){
	if(loggerEnabled){
	        double heading;
        	double pitch;
	        double roll;

        	QuaternionUtils::convertToEulerAngles(odom.pose.pose.orientation,heading,pitch,roll);

		fprintf(imuOutputFile,"%s%s%.3f%s%.3f%s%.3f\n",TimeUtils::getTimestampString(odom.header.stamp.sec, odom.header.stamp.nsec).c_str(),separator.c_str(),R2D(heading),separator.c_str(),R2D(pitch),separator.c_str(),R2D(roll));
	}
}

void Writer::sonarCallback(const geometry_msgs::PointStamped& sonar){
	if(loggerEnabled){
		fprintf(sonarOutputFile,"%s%s%.3f\n",TimeUtils::getTimestampString(sonar.header.stamp.sec, sonar.header.stamp.nsec).c_str(),separator.c_str(),sonar.point.z);
	}
}

bool Writer::getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response){
	response.status = loggerEnabled;
	return true;
}


bool Writer::toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response){
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

	mtx.unlock();

	response.loggingStatus=loggerEnabled;



	return true;
}
