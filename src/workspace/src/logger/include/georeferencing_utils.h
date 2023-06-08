#include "types.h"
#include <cmath>
#include <Eigen/Dense>

#include "lidarFilters.h"

#define PI M_PI
#define INF 1.e100
#define R2D ((double)180/(double)PI)
#define D2R ((double)PI/(double)180)
#define a_wgs84 6378137.0
#define e2_wgs84 (0.081819190842622 * 0.081819190842622)


namespace Interpolator{
	
		double linearInterpolationByTime(double y1, double y2, uint64_t targetTimestamp, uint64_t t1, uint64_t t2) {
			if (t1 == t2){
				//throw new Exception("The two positions timestamp are the same");
			}
			if (t1 > targetTimestamp){
				//throw new Exception("The first position timestamp is higher than interpolation timestamp");
			}
			if (t1 > t2){
				//throw new Exception("The first position timestamp is higher than the second position timestamp");
			}
			
			double result = (y1 + (y2 - y1)*(targetTimestamp - t1) / (t2 - t1));
			return result;
	}
	
	double linearAngleInterpolationByTime(double a1, double a2, uint64_t targetTimestamp, uint64_t t1, uint64_t t2) {

		if (t1 == t2){
			//throw new Exception("The two positions timestamp are the same");
		}
		if (t1 > targetTimestamp){
			//throw new Exception("The first position timestamp is higher than interpolation timestamp");
		}
		if (t1 > t2){
			//throw new Exception("The first position timestamp is higher than the second position timestamp");
		}

		if (std::abs(a2 - a1)==180){
			std::stringstream ss;        
			ss << "The angles " << a1 << " and " << a2
			<< " have a difference of 180 degrees which means there are two possible answers at timestamp " << targetTimestamp;
			//throw new Exception(ss.str());
			
		}

		if (a1 == a2) {
			return a1;
		}

		double x1 = targetTimestamp-t1;
		double x2 = t2-t1;
		double delta = (x1 / x2);
		double dpsi = std::fmod((std::fmod(a2 - a1, 360) + 540), 360) - 180;

		double total = a1 + dpsi*delta;

		if(total > 0){
			return (total < 360.0)? total : fmod(total,360.0);
		}
		else{
			return total + 360.0; //TODO: handle angles -360....-520...etc
		}
  }

	
	PositionPacket* interpolatePosition(std::pair<PacketHeader, PositionPacket> & beforePosition, 
										std::pair<PacketHeader, PositionPacket> & afterPosition, uint64_t targetTimestamp){
		
		uint64_t beforeTimestamp = std::get<PacketHeader>(beforePosition).packetTimestamp;
		uint64_t afterTimestamp = std::get<PacketHeader>(afterPosition).packetTimestamp;


		PositionPacket *interpolatedPosition = new PositionPacket;
		
		interpolatedPosition->latitude  = linearInterpolationByTime(std::get<PositionPacket>(beforePosition).latitude, std::get<PositionPacket>(afterPosition).latitude,
							 targetTimestamp, beforeTimestamp, afterTimestamp);
		
		interpolatedPosition->longitude = linearInterpolationByTime(std::get<PositionPacket>(beforePosition).longitude, std::get<PositionPacket>(afterPosition).longitude,
							 targetTimestamp, beforeTimestamp, afterTimestamp);

		interpolatedPosition->altitude = linearInterpolationByTime(std::get<PositionPacket>(beforePosition).altitude, std::get<PositionPacket>(afterPosition).altitude,
							 targetTimestamp, beforeTimestamp, afterTimestamp);

		return interpolatedPosition;
	}
	
	AttitudePacket* interpolateAttitude(std::pair<PacketHeader, AttitudePacket> & beforePosition, 
										std::pair<PacketHeader, AttitudePacket> & afterPosition, uint64_t targetTimestamp){
	
		uint64_t beforeTimestamp = std::get<PacketHeader>(beforePosition).packetTimestamp;
		uint64_t afterTimestamp = std::get<PacketHeader>(afterPosition).packetTimestamp;
		
		AttitudePacket *interpolatedAttitude = new AttitudePacket;
		
		interpolatedAttitude->heading = linearInterpolationByTime(std::get<AttitudePacket>(beforePosition).heading, std::get<AttitudePacket>(afterPosition).heading,
							 targetTimestamp, beforeTimestamp, afterTimestamp);
		
		interpolatedAttitude->pitch = linearInterpolationByTime(std::get<AttitudePacket>(beforePosition).pitch, std::get<AttitudePacket>(afterPosition).pitch,
							 targetTimestamp, beforeTimestamp, afterTimestamp);

		interpolatedAttitude->roll = linearInterpolationByTime(std::get<AttitudePacket>(beforePosition).roll, std::get<AttitudePacket>(afterPosition).roll,
							 targetTimestamp, beforeTimestamp, afterTimestamp);

		return interpolatedAttitude;
										
	}
	
}

namespace Georeference{
		
	void generateNedToEcefMatrix(Eigen::Matrix3d & outputMatrix, PositionPacket & position){
	
		outputMatrix << -sin(position.latitude*D2R)*cos(position.longitude*D2R), -sin(position.longitude*D2R), -cos(position.latitude*D2R)*cos(position.longitude*D2R),
					-sin(position.latitude*D2R) * sin(position.longitude*D2R), cos(position.longitude*D2R), -cos(position.latitude*D2R)*sin(position.longitude*D2R),
					cos(position.latitude*D2R), 0, -sin(position.latitude*D2R);
					
	}
	
	void generateDcmMatrix(Eigen::Matrix3d & outputMatrix, AttitudePacket & attitude){
		
		double ch = cos(attitude.heading*D2R);
		double sh = sin(attitude.heading*D2R);
		double cp = cos(attitude.pitch*D2R);
		double sp = sin(attitude.pitch*D2R);
		double cr = cos(attitude.roll*D2R);
		double sr = sin(attitude.roll*D2R);

		outputMatrix << 	ch*cp , ch*sp*sr - sh*cr  , ch*sp*cr + sh*sr,
		sh*cp , sh*sp*sr + ch*cr  , sh*sp*cr - ch*sr,
		-sp   , cp*sr		  , cp*cr;
		
	}
	
	void generateEcefToNed(Eigen::Matrix3d & outputMatrix, double & firstLat, double & firstLon){
		// ecef to ned matrix
		outputMatrix << -sin(firstLat*D2R)*cos(firstLon*D2R), -sin(firstLat*D2R) * sin(firstLon*D2R), cos(firstLat*D2R),
					-sin(firstLon*D2R), cos(firstLon*D2R), 0,
					-cos(firstLat*D2R)*cos(firstLon*D2R), -cos(firstLat*D2R)*sin(firstLon*D2R), -sin(firstLat*D2R);
	}
	
	
	void getPositionECEF(Eigen::Vector3d & positionECEF, PositionPacket & position){
		double N = a_wgs84  / (sqrt(1 - e2_wgs84 * sin(position.latitude * D2R) *  sin(position.latitude * D2R)));
		double xTRF = (N + position.altitude) * cos(position.latitude * D2R) * cos(position.longitude * D2R);
		double yTRF = (N + position.altitude) * cos(position.latitude * D2R) * sin(position.longitude * D2R);
		double zTRF = (N * (1 - e2_wgs84) + position.altitude) * sin(position.latitude * D2R);
		positionECEF << xTRF, yTRF, zTRF;
	}
	
	void georeferenceLGF(Eigen::Vector3d & georeferencedLaserPoint, Eigen::Matrix3d & ecefToNed, PositionPacket & firstPosition, AttitudePacket & attitude, PositionPacket & position, LidarPacket & point, Eigen::Vector3d & leverArm, Eigen::Matrix3d & boresight) {
		
    	
    	Eigen::Matrix3d imu2ned;
    	generateDcmMatrix(imu2ned, attitude);
    	
    	Eigen::Vector3d positionECEF;
    	getPositionECEF(positionECEF, position);
    	
    	Eigen::Vector3d firstPositionECEF;
    	getPositionECEF(firstPositionECEF, firstPosition);
    	
    	Eigen::Vector3d positionNed = ecefToNed * (positionECEF -  firstPositionECEF);
    	
    	Eigen::Vector3d lidarPoint(point.laser_y, point.laser_x, -point.laser_z);
		Eigen::Vector3d laserPointNed = imu2ned * (boresight * lidarPoint); //
		
		//Convert lever arm to NED
		Eigen::Vector3d leverArmNed = imu2ned * leverArm;
		
		georeferencedLaserPoint = positionNed + laserPointNed + leverArmNed;
		
		
		std::cout<<georeferencedLaserPoint(0)<<" "<<georeferencedLaserPoint(1)<<" "<< georeferencedLaserPoint(2) << std::endl;
		
		}
	
}
