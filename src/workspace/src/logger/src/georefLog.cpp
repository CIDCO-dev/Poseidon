#include "poseidon_reader.h"
#include "georeferencing_utils.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <stdio.h>
#include <unistd.h>
#include <SbetProcessor.hpp>
#include <filesystem>
#include <boost/progress.hpp>

class PoseidonBinaryLidarGeoref : public PoseidonBinaryReader, public SbetProcessor{
	public:
		PoseidonBinaryLidarGeoref(std::string & filePath, Eigen::Vector3d &leverArm, Eigen::Matrix3d &boresight, std::string sbetFilePath) : PoseidonBinaryReader(filePath), leverArm(leverArm), boresight(boresight), sbetFilePath(sbetFilePath){}
		~PoseidonBinaryLidarGeoref(){}
		
		void processEntry(SbetEntry * entry){
			
			PacketHeader hdrPosition;
			hdrPosition.packetTimestamp = static_cast<uint64_t>(entry->time * 1000000);
			hdrPosition.packetType = PACKET_POSITION;
			hdrPosition.packetSize = sizeof(PositionPacket);
			
			//printf("%.8f\n",entry->time);
			
			PositionPacket position;
			position.longitude = (entry->longitude* 180) / M_PI;
			position.latitude = (entry->latitude * 180) / M_PI;
			position.altitude = entry->altitude;
			
			std::pair pairPosition = std::make_pair(hdrPosition, position);
			positions.push_back(pairPosition);
			
			
			PacketHeader hdrAttitude;
			hdrAttitude.packetTimestamp = static_cast<uint64_t>(entry->time * 1000000);
			hdrAttitude.packetType = PACKET_ATTITUDE;
			hdrAttitude.packetSize = sizeof(AttitudePacket);
			
			
			AttitudePacket attitude;
			attitude.roll = entry->roll*180/M_PI;
			attitude.pitch = entry->pitch*180/M_PI;
			attitude.heading = entry->heading*180/M_PI;
			
			std::pair pairAttitude = std::make_pair(hdrAttitude, attitude);
			attitudes.push_back(pairAttitude);
			
			
		}
		void done(){
		
		}
		
		void processGnss(PacketHeader & hdr, PositionPacket & packet)override{
			std::pair pairPacket = std::make_pair(hdr, packet);
			positions.push_back(pairPacket);
		}
		
		void processImu(PacketHeader & hdr, AttitudePacket & packet)override{
			std::pair pairPacket = std::make_pair(hdr, packet);
			attitudes.push_back(pairPacket);
		}
		
		void processLidar(PacketHeader & hdr, LidarPacket &packet)override{
			std::pair pairPacket = std::make_pair(hdr, packet);
			laserPoints.push_back(pairPacket);
		}
		
		std::vector<std::pair<PacketHeader, PositionPacket>> getPositions(){
			return positions;
		}
		
		
		std::vector<std::pair<PacketHeader, AttitudePacket>> getAtitudes(){
			return attitudes;
		}
		
		
		std::vector<std::pair<PacketHeader, LidarPacket>> getLaserPoints(){
			return laserPoints;
		}
		
		void setFilter(double &minAngle, double &maxAngle, double &minDistance, double &maxDistance){
			this->activatedFilter = true;
			this->minAngle = minAngle;
			this->maxAngle = maxAngle;
			this->minDistance = minDistance;
			this->maxDistance = maxDistance;
		}
		
		void georeference(){
			
			// if we have sbet read it
			if(sbetFilePath.size() > 0){
				std::cerr<<"[+] Using Sbet for georeferencing" << std::endl;
				positions.clear();
				attitudes.clear();
				readFile(sbetFilePath);
			}
		
		
			if(this->positions.size()==0){
				std::cerr << "[-] No position data found in file" << std::endl;
				return;
			}

			if(this->attitudes.size()==0){
				std::cerr << "[-] No attitude data found in file" << std::endl;
				return;
			}

			if(this->laserPoints.size()==0){
				std::cerr << "[-] No laserPoints data found in file" << std::endl;
				return;
			}
			
			std::cerr<<this->laserPoints.size() << " points read"<<std::endl; 
			
			// TODO Compute centroid or use first position
			
			//Sort everything
			std::sort(positions.begin(), positions.end(), &sortByTimestamp<PositionPacket>);
			std::sort(attitudes.begin(), attitudes.end(), &sortByTimestamp<AttitudePacket>);
			std::sort(laserPoints.begin(), laserPoints.end(), &sortByTimestamp<LidarPacket>);
			
			std::cerr<<"sorting by timestamp done."<<std::endl;
			
			//fix timeStamps
			if(sbetFilePath.size() > 0){
				uint64_t attitudeTimestamp = std::get<PacketHeader>(attitudes[0]).packetTimestamp;
				uint64_t positionTimestamp = std::get<PacketHeader>(positions[0]).packetTimestamp;
				uint64_t laserPointTimestamp = std::get<PacketHeader>(laserPoints[0]).packetTimestamp;
				
				std::cerr << "first point : " << laserPointTimestamp <<"\n";
				
				if( attitudeTimestamp == positionTimestamp){
					
					// unix time 1st jan 1970 is a thursday
					// gps first day of the week starts on sundays
					// 4 day difference : 60 *60 *24 *4 = 345600 seconds
					uint64_t offset = 345600000000;
					
					uint64_t nbMicroSecondsPerWeek = 604800000000; // microseconds per week
					
					//std::cerr<<"micro sec per week : " << nbMicroSecondsPerWeek <<"\n";
					
					uint64_t nbWeek = (laserPointTimestamp + (nbMicroSecondsPerWeek - offset)) / nbMicroSecondsPerWeek; //nb week unix time
					
					std::cerr<<"nb week : " << nbWeek << "\n";
					
					uint64_t startOfWeek = (nbMicroSecondsPerWeek * nbWeek) - offset; // micro sec from unix time to start of week
					
					for (auto i = laserPoints.begin(); i != laserPoints.end(); i++) {
						std::get<PacketHeader>(*i).packetTimestamp = std::get<PacketHeader>(*i).packetTimestamp - startOfWeek;
					}
				}
				else{
					std::cerr<<"Sbet imu & gnss timestamp are not the same \n";
					exit(1);
				}
				
			}
			
			
			// For correct display of timestamps on Windows
			std::cerr <<  "[+] Position data points: " << positions.size() << " [" << std::get<PacketHeader>(positions[0]).packetTimestamp << " to " 
					<< std::get<PacketHeader>(positions[positions.size() - 1]).packetTimestamp << "]\n";
					
			std::cerr <<  "[+] Attitude data points: " << attitudes.size() << " [" << std::get<PacketHeader>(attitudes[0]).packetTimestamp << " to " 
					<< std::get<PacketHeader>(attitudes[attitudes.size() - 1]).packetTimestamp << "]\n";  
						
			std::cerr <<  "[+] Ping data points: " << laserPoints.size() << " [" 
					<< ( (laserPoints.size() > 0) ? std::get<PacketHeader>(laserPoints[0]).packetTimestamp : 0 ) << " to " 
					<< ( (laserPoints.size() > 0) ? std::get<PacketHeader>(laserPoints[laserPoints.size() - 1]).packetTimestamp : 0 ) << "]\n";
			
			
			//interpolate attitudes and positions around pings
			unsigned int attitudeIndex = 0;
			unsigned int positionIndex = 0;
			
			PositionPacket firstPosition = std::get<PositionPacket>(positions[0]);
			
			double firstLat = firstPosition.latitude;
			double firstLon = firstPosition.longitude;
			
			Eigen::Matrix3d ecefToNed;
			Georeference::generateEcefToNed(ecefToNed, firstLat, firstLon);
			
			boost::progress_display progress(this->laserPoints.size(), std::cerr);
			
			//filter laser points
			for (auto i = laserPoints.begin(); i != laserPoints.end(); i++) {
				
				++progress;
				
				if(this->activatedFilter){
				
					LidarPacket point = std::get<LidarPacket>(*i);
					
					if(Filters::distanceFilter(point.laser_x, point.laser_y, point.laser_z, this->minDistance, this->maxDistance) ||
					   Filters::horizontalAngleFilter(point.laser_x, point.laser_y, this->minAngle, this->maxAngle))
					{
						continue;
					}
				}
				
				
				while (attitudeIndex + 1 < attitudes.size() && 
						std::get<PacketHeader>(attitudes[attitudeIndex + 1]).packetTimestamp < std::get<PacketHeader>(*i).packetTimestamp) 
				{	
					attitudeIndex++;
				}

				//No more attitudes available
				if (attitudeIndex >= attitudes.size() - 1) {
					std::cerr << "No more attitudes" << std::endl;
					break;
				}
					
					
				while (positionIndex + 1 < positions.size() && 
						std::get<PacketHeader>(positions[positionIndex + 1]).packetTimestamp < std::get<PacketHeader>(*i).packetTimestamp) 
				{
					positionIndex++;
				}
				
				//No more positions available
				if (positionIndex >= positions.size() - 1) {
					std::cerr << "No more positions" << std::endl;
					break;
				}
				
				//No position or attitude smaller than ping, so discard this ping
				if (std::get<PacketHeader>(positions[positionIndex]).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp 
					|| std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp)
					
				{
					
					// this commented block can create segFault if filter is activated
					/*
					int nbPoints = std::get<PacketHeader>(*i).packetSize / sizeof(LidarPacket);
					
					std::cerr << "rejecting " << nbPoints << " lidar point " << std::get<PacketHeader>(*i).packetTimestamp << " " 
							<< std::get<PacketHeader>(positions[positionIndex]).packetTimestamp << " "
							<< std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp << std::endl;
					
					for(int lidarPoint = 0; lidarPoint < nbPoints-1; ++lidarPoint){
						i++;
					}
					*/
					
					// no segfault if filter is activated but its much slower
					std::cerr << "rejecting lidar point " << std::get<PacketHeader>(*i).packetTimestamp << " " 
							<< std::get<PacketHeader>(positions[positionIndex]).packetTimestamp << " "
							<< std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp << std::endl;
					
					continue;
				}
				
				
				AttitudePacket * interpolatedAttitude = Interpolator::interpolateAttitude(attitudes[attitudeIndex], 
																					  attitudes[attitudeIndex + 1], 
																					  std::get<PacketHeader>(*i).packetTimestamp);
				
				
				PositionPacket * interpolatedPosition = Interpolator::interpolatePosition(positions[positionIndex], 
																						  positions[positionIndex + 1],
													 									  std::get<PacketHeader>(*i).packetTimestamp);
				
				
				//georeference
				Eigen::Vector3d georeferencedLaserPoint;
				
				
				Georeference::georeferenceLGF(georeferencedLaserPoint, ecefToNed, firstPosition, *interpolatedAttitude, *interpolatedPosition, std::get<LidarPacket>(*i), leverArm, boresight);
				
				
				delete interpolatedAttitude;
				delete interpolatedPosition;
			}
		}
		
		template <typename T>
		static bool sortByTimestamp(std::pair<PacketHeader, T> & pair1, std::pair<PacketHeader, T> & pair2){
			PacketHeader header1 = std::get<PacketHeader>(pair1);
			PacketHeader header2 = std::get<PacketHeader>(pair2);
	
			return header1.packetTimestamp < header2.packetTimestamp;
		}
		
		
	private:
		
		std::vector<std::pair<PacketHeader, PositionPacket>> positions;
		std::vector<std::pair<PacketHeader, AttitudePacket>> attitudes;
		std::vector<std::pair<PacketHeader, LidarPacket>>  laserPoints;
		Eigen::Vector3d leverArm;
		Eigen::Matrix3d boresight;
		double minAngle = 0.0;
		double maxAngle = 0.0;
		double minDistance = 0.0;
		double maxDistance = 0.0;
		bool activatedFilter = false;
		std::string sbetFilePath="";

};

void printUsage(){
	std::cerr << "\n\
NAME\n\n\
	georeference - Produces a georeferenced point cloud from binary lidar datagrams files\n\n\
SYNOPSIS\n \
	georeference [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] [-s sbet_file] file\n\n\
DESCRIPTION\n \
	-f Activate lidar filtering\n \
	-a Minimum angle\n \
	-b Maximum angle\n \
	-d Minimum distance\n \
	-e Maximum distance\n \
Copyright 2017-2023 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;
	exit(1);
}


int main(int argc,char** argv){

	if(argc < 2){
		printUsage();
	}
	else{
		std::string fileName(argv[argc-1]);

		//Lever arm
		double leverArmX = 0.0;
		double leverArmY = 0.0;
		double leverArmZ = 0.0;

		//Boresight
		double roll	 = 0.0;
		double pitch	= 0.0;
		double heading  = 0.0;
		
		bool activateFilter = false;
		double minAngle = 0.0;
		double maxAngle = 0.0;
		double minDistance = 0.0;
		double maxDistance = 0.0;
		
		char sbetFilePath[1024] = "";

		int index;

		while((index=getopt(argc,argv,"a:b:d:e:x:y:z:r:p:h:s:f"))!=-1){
			switch(index){
				case 'x':
				if(sscanf(optarg,"%lf", &leverArmX) != 1){
					std::cerr << "Invalid lever arm X offset (-x)" << std::endl;
					printUsage();
				}
				break;

				case 'y':
				if (sscanf(optarg,"%lf", &leverArmY) != 1){
					std::cerr << "Invalid lever arm Y offset (-y)" << std::endl;
					printUsage();
				}
				break;

				case 'z':
				if (sscanf(optarg,"%lf", &leverArmZ) != 1){
					std::cerr << "Invalid lever arm Z offset (-z)" << std::endl;
					printUsage();
				}
				break;

				case 'r':
				if (sscanf(optarg,"%lf", &roll) != 1){
					std::cerr << "Invalid roll angle offset (-r)" << std::endl;
					printUsage();
				}
				break;

				case 'h':
				if (sscanf(optarg,"%lf", &heading) != 1){
					std::cerr << "Invalid heading angle offset (-h)" << std::endl;
					printUsage();
				}
				break;

				case 'p':
				if (sscanf(optarg,"%lf", &pitch) != 1){
					std::cerr << "Invalid pitch angle offset (-p)" << std::endl;
					printUsage();
				}
				break;
				
				case 'f':
					activateFilter = true;
				break;
				
				case 'a':
				if (sscanf(optarg,"%lf", &minAngle) != 1){
					std::cerr << "Invalid minimum angle (-a)" << std::endl;
					printUsage();
				}
				break;
				
				case 'b':
				if (sscanf(optarg,"%lf", &maxAngle) != 1){
					std::cerr << "Invalid maximum angle (-b)" << std::endl;
					printUsage();
				}
				break;
				
				case 'd':
				if (sscanf(optarg,"%lf", &minDistance) != 1){
					std::cerr << "Invalid minimum distance (-d)" << std::endl;
					printUsage();
				}
				break;
				
				case 'e':
				if (sscanf(optarg,"%lf", &maxDistance) != 1){
					std::cerr << "Invalid maximum distance (-e)" << std::endl;
					printUsage();
				}
				break;
				
				case 's':
				if (sscanf(optarg,"%s", sbetFilePath) != 1){
					std::cerr << "Invalid maximum distance (-e)" << std::endl;
					printUsage();
				}
				else{
					std::filesystem::path f{sbetFilePath};
					if (!std::filesystem::exists(f)){
						std::cerr << "Sbet file path provided does not exists" << std::endl;
						printUsage();
					} 
				}
				break;
				
			}	
		}
		
	//Lever arm
	Eigen::Vector3d leverArm;
	leverArm << leverArmX, leverArmY, leverArmZ;
	
	//Boresight
	Eigen::Matrix3d boresightMatrix;
	
	AttitudePacket boresight;
	boresight.roll = roll;
	boresight.pitch = pitch;
	boresight.heading = heading;
	Georeference::generateDcmMatrix(boresightMatrix, boresight);
	
	PoseidonBinaryLidarGeoref georeferencer(fileName, leverArm, boresightMatrix, sbetFilePath);
	
	if(activateFilter){
		georeferencer.setFilter(minAngle, maxAngle, minDistance, maxDistance);
	}
	
	georeferencer.read();
	georeferencer.georeference();
	}
	
	return 0;
}
