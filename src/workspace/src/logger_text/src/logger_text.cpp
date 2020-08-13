#include "logger_text/logger_text.h"
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"

Writer::Writer(std::string & gnssFilePath, std::string & imuFilePath, std::string & sonarFilePath, std::string separator):gnssFilePath(gnssFilePath),imuFilePath(imuFilePath),sonarFilePath(sonarFilePath),separator(separator){
	init();
}

Writer::~Writer(){
	if(gnssOutputFile.is_open())   gnssOutputFile.close();
	if(imuOutputFile.is_open())    imuOutputFile.close();
	if(sonarOutputFile.is_open())  sonarOutputFile.close();
}

void Writer::init(){
	gnssOutputFile.open(gnssFilePath);

	if(!gnssOutputFile.is_open()){
		throw std::invalid_argument(std::string("Couldn't open GNSS log file ") + gnssFilePath);
	}

	imuOutputFile.open(imuFilePath);

        if(!imuOutputFile.is_open()){
	        throw std::invalid_argument(std::string("Couldn't open IMU log file ") + imuFilePath);
        }

	sonarOutputFile.open(sonarFilePath);

        if(!sonarOutputFile.is_open()){
 	       throw std::invalid_argument(std::string("Couldn't open sonar log file ") + sonarFilePath);
        }

	gnssOutputFile  << "TimeStamp"
                        << separator << "Longitude"
                        << separator << "Latitude"
                        << separator << "EllipsoidalHeight"
                        << std::endl;

	double heading;
	double pitch;
	double roll;

	

        imuOutputFile   << "TimeStamp"
                        << separator << "Heading"
                        << separator << "Pitch"
                        << separator << "Roll"
                        << std::endl;

        sonarOutputFile << "TimeStamp"
                        << separator << "Depth"
                        << std::endl;
}

void Writer::gnssCallback(const sensor_msgs::NavSatFix& gnss){
	gnssOutputFile  << TimeUtils::buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec)
                        << separator << gnss.longitude
                        << separator << gnss.latitude
                        << separator << gnss.altitude
                        << std::endl;
}

void Writer::imuCallback(const nav_msgs::Odometry& odom){
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

void Writer::sonarCallback(const geometry_msgs::PointStamped& sonar){
                sonarOutputFile << TimeUtils::buildTimeStamp(sonar.header.stamp.sec, sonar.header.stamp.nsec)
                                << separator << sonar.point.z
                                << std::endl;
}
