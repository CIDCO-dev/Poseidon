#include "poseidon_reader.h"
#include <SbetProcessor.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "georeferencing_utils.h"

class CalibPrinter : public PoseidonBinaryReader, public SbetProcessor{
	public:
		CalibPrinter(std::string & filePath, std::string sbetFilePath, Eigen::Vector3d &_leverArm) : PoseidonBinaryReader(filePath), sbetFilePath(sbetFilePath),
																									leverArm(_leverArm){}
		~CalibPrinter(){}
		
		void processEntry(SbetEntry * entry){
			
			PacketHeader hdrPosition;
			hdrPosition.packetTimestamp = static_cast<uint64_t>(entry->time * 1000000.0);
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
			hdrAttitude.packetTimestamp = static_cast<uint64_t>(entry->time * 1000000.0);
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
		
		
				template <typename T>
		static bool sortByTimestamp(std::pair<PacketHeader, T> & pair1, std::pair<PacketHeader, T> & pair2){
			PacketHeader header1 = std::get<PacketHeader>(pair1);
			PacketHeader header2 = std::get<PacketHeader>(pair2);
	
			return header1.packetTimestamp < header2.packetTimestamp;
		}
		
		
		void print(){
			positions.clear();
			attitudes.clear();
			readFile(sbetFilePath);
			
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
			
			std::cerr<<this->positions.size() <<" position \n";
			std::cerr<<this->attitudes.size() <<" attitudes \n";
			std::cerr<<this->laserPoints.size() <<" laserPoints \n";
			
			//Sort everything
			std::sort(positions.begin(), positions.end(), &sortByTimestamp<PositionPacket>);
			std::sort(attitudes.begin(), attitudes.end(), &sortByTimestamp<AttitudePacket>);
			std::sort(laserPoints.begin(), laserPoints.end(), &sortByTimestamp<LidarPacket>);
			
			std::cerr<<"sorting done \n";
			
			//fix timeStamps
			if(sbetFilePath.size() > 0){
				uint64_t attitudeTimestamp = std::get<PacketHeader>(attitudes[0]).packetTimestamp;
				uint64_t positionTimestamp = std::get<PacketHeader>(positions[0]).packetTimestamp;
				uint64_t laserPointTimestamp = std::get<PacketHeader>(laserPoints[0]).packetTimestamp;
				
				std::cerr << "first laser point unix time: " << laserPointTimestamp <<"\n";
				std::cerr << "first postion gps time: " << positionTimestamp <<"\n";
				
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
					
					std::cerr<<"start of week unix time: " << startOfWeek<<"\n";
					
					for (auto i = laserPoints.begin(); i != laserPoints.end(); i++) {
						std::get<PacketHeader>(*i).packetTimestamp = std::get<PacketHeader>(*i).packetTimestamp - startOfWeek;
					}
				}
				else{
					std::cerr<<"Sbet imu & gnss timestamp are not the same \n";
					exit(1);
				}
			}
			
			//interpolate attitudes and positions around pings
			unsigned int attitudeIndex = 0;
			unsigned int positionIndex = 0;
			
			PositionPacket firstPosition = std::get<PositionPacket>(positions[0]);
			double firstLat = firstPosition.latitude;
			double firstLon = firstPosition.longitude;
			
			Eigen::Matrix3d ecefToNed;
			Georeference::generateEcefToNed(ecefToNed, firstLat, firstLon);
			
			Eigen::Matrix3d nedToEcef;
			Georeference::generateNedToEcef(nedToEcef, firstLat, firstLon);
			
			Eigen::Matrix3d aligmentMatrix;
			AttitudePacket aligment;
			aligment.roll = 0.0;
			aligment.pitch = 45.0;
			aligment.heading = 180.0;
			Georeference::generateDcmMatrix(aligmentMatrix, aligment);
			
			
			for (auto i = laserPoints.begin(); i != laserPoints.end(); i++) {
				
				uint64_t seconds = std::get<PacketHeader>(*i).packetTimestamp /1000000;
				uint64_t residual = std::get<PacketHeader>(*i).packetTimestamp - 1000000*seconds;
				
				double timestamp = seconds + residual/1000000.0;
				
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
				
				//No position or attitude smaller than beam, discard this beam
				if (std::get<PacketHeader>(positions[positionIndex]).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp 
					|| std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp)
					
				{
					int nbPoints = std::get<PacketHeader>(*i).packetSize / sizeof(LidarPacket);
					
					std::cerr << "rejecting " << nbPoints << " lidar point " << std::get<PacketHeader>(*i).packetTimestamp << " " 
							<< std::get<PacketHeader>(positions[positionIndex]).packetTimestamp << " "
							<< std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp << std::endl;
					
					for(int lidarPoint = 0; lidarPoint < nbPoints-1; ++lidarPoint){
						i++;
					}
					
					
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
				Georeference::georeferenceECEF(georeferencedLaserPoint, firstPosition, *interpolatedAttitude, *interpolatedPosition, std::get<LidarPacket>(*i), leverArm, aligmentMatrix, nedToEcef, ecefToNed);
				
				std::cout<<std::setprecision(6)<<std::get<LidarPacket>(*i).laser_x <<" "<<std::get<LidarPacket>(*i).laser_y <<" "<<std::get<LidarPacket>(*i).laser_z <<" 0 "<<std::setprecision(16)<< timestamp << " "<< georeferencedLaserPoint(0) <<" "<< georeferencedLaserPoint(1)<<" "<< georeferencedLaserPoint(2)<< "\n";
				
				
				delete interpolatedAttitude;
				delete interpolatedPosition;
			}
		}
		
		
	private:
		
		std::vector<std::pair<PacketHeader, PositionPacket>> positions;
		std::vector<std::pair<PacketHeader, AttitudePacket>> attitudes;
		std::vector<std::pair<PacketHeader, LidarPacket>>  laserPoints;
		std::string sbetFilePath="";
		Eigen::Vector3d leverArm;
		
	};


int main(int argc,char** argv){

	if(argc != 6){
		std::cerr << "calib_printer filePath sbetFilePath leverArmX leverArmY leverArmZ" << std::endl;
		exit(1);
	}
	std::string poseidonFilePath = argv[1];
	std::string sbetFilePath = argv[2];
	
	//TODO TRY block
	
	double leverArmX = std::stod(argv[3]);
	double leverArmY = std::stod(argv[4]);
	double leverArmZ = std::stod(argv[5]);
	
	//Lever arm
	Eigen::Vector3d leverArm(leverArmX, leverArmY, leverArmZ);
	
	CalibPrinter printer(poseidonFilePath, sbetFilePath, leverArm);
	printer.read();
	printer.print();
	
	return 0;
}
