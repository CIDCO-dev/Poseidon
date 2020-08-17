#include "logger_text/logger_text.h"
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"

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
	std::string gnssFileName =  outputFolder + "/"  + dateString + "_gnss.txt";

	gnssOutputFile.open(gnssFileName);

	if(!gnssOutputFile.is_open()){
		throw std::invalid_argument(std::string("Couldn't open GNSS log file ") + gnssFileName);
	}

	//Open IMU file
	std::string imuFileName = outputFolder + "/" + dateString + "_imu.txt";

	imuOutputFile.open(imuFileName);

        if(!imuOutputFile.is_open()){
	        throw std::invalid_argument(std::string("Couldn't open IMU log file ") + imuFileName);
        }

	//Open sonar file
	std::string sonarFileName = outputFolder + "/" + dateString + "_sonar.txt";

	sonarOutputFile.open(sonarFileName);

        if(!sonarOutputFile.is_open()){
 	       throw std::invalid_argument(std::string("Couldn't open sonar log file ") + sonarFileName);
        }

	gnssOutputFile  << "TimeStamp"
                        << separator << "Longitude"
                        << separator << "Latitude"
                        << separator << "EllipsoidalHeight"
                        << std::endl;

        imuOutputFile   << "TimeStamp"
                        << separator << "Heading"
                        << separator << "Pitch"
                        << separator << "Roll"
                        << std::endl;

        sonarOutputFile << "TimeStamp"
                        << separator << "Depth"
                        << std::endl;
}

void Writer::finalize(){
        if(gnssOutputFile.is_open())   gnssOutputFile.close();
        if(imuOutputFile.is_open())    imuOutputFile.close();
        if(sonarOutputFile.is_open())  sonarOutputFile.close();
}

void Writer::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	if(loggerEnabled){
		gnssOutputFile  << TimeUtils::buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec)
        	                << separator << gnss.longitude
                	        << separator << gnss.latitude
                        	<< separator << gnss.altitude
                        	<< std::endl;
	}
}

void Writer::imuCallback(const nav_msgs::Odometry& odom){
	if(loggerEnabled){
	        double heading;
        	double pitch;
	        double roll;

        	QuaternionUtils::convertToEulerAngles(odom.pose.pose.orientation,heading,pitch,roll);

        	imuOutputFile   << TimeUtils::buildTimeStamp(odom.header.stamp.sec, odom.header.stamp.nsec)
                                << separator << R2D(heading)
                                << separator << R2D(pitch)
                                << separator << R2D(roll)
                                << std::endl;
	}
}

void Writer::sonarCallback(const geometry_msgs::PointStamped& sonar){
	if(loggerEnabled){
                sonarOutputFile << TimeUtils::buildTimeStamp(sonar.header.stamp.sec, sonar.header.stamp.nsec)
                                << separator << sonar.point.z
                                << std::endl;
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
