#include "poseidon_reader.h"
#include "georeferencing_utils.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <stdio.h>
#include <unistd.h>

class PoseidonBinaryLidarGeoref : public PoseidonBinaryReader{
	public:
		PoseidonBinaryLidarGeoref(std::string & filePath, Eigen::Vector3d &leverArm, Eigen::Matrix3d &boresight) : PoseidonBinaryReader(filePath), 
																													leverArm(leverArm), boresight(boresight){}
		~PoseidonBinaryLidarGeoref(){}
		
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
		
		
		void georeference(){
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
        	
        	//Georef pings
        	for (auto i = laserPoints.begin(); i != laserPoints.end(); i++) {
        	
        		/*
        		if(! LidarFilter::keepPoint(std::get<LidarPacket>(*i),-180.0, , 1.0, 40.0)){
        			continue;
        		}
        		*/
        		/*
        		if (LidarFilter::badPoint(std::get<LidarPacket>(*i), -60.0, 60.0, 1.5, 20.0)){
        			continue;
        		}
        		*/
        		/*
        		if(LidarFilter::azimutFilter(std::get<LidarPacket>(*i),0, 180) || LidarFilter::distanceFilter(std::get<LidarPacket>(*i),1.0,60.0)){
        			continue;	
        		}
        		*/
        		/*
        		if(LidarFilter::azimutFilter(std::get<LidarPacket>(*i),-180, 0)){
        			continue;	
        		}
        		*/
        		
        		if(LidarFilter::distanceFilter(std::get<LidarPacket>(*i),2.0, 50.0)){
        			continue;
        		}
        		
        		
        		while (attitudeIndex + 1 < attitudes.size() && 
						std::get<PacketHeader>(attitudes[attitudeIndex + 1]).packetTimestamp < std::get<PacketHeader>(*i).packetTimestamp) 
				{	
		            attitudeIndex++;
		        }

		        //No more attitudes available
		        if (attitudeIndex >= attitudes.size() - 1) {
		            //std::cerr << "No more attitudes" << std::endl;
		            break;
		        }

		        while (positionIndex + 1 < positions.size() && 
						std::get<PacketHeader>(positions[positionIndex + 1]).packetTimestamp < std::get<PacketHeader>(*i).packetTimestamp) 
				{
		            positionIndex++;
		        }
		        
		        //No more positions available
		        if (positionIndex >= positions.size() - 1) {
		            //std::cerr << "No more positions" << std::endl;
		            break;
		        }
		        
		        //No position or attitude smaller than ping, so discard this ping
            	if (std::get<PacketHeader>(positions[positionIndex]).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp 
					|| std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp)
					
					{
            		
            		int nbPoints = std::get<PacketHeader>(*i).packetSize / sizeof(LidarPacket);
            		
            		std::cerr << "rejecting " << nbPoints << " lidar point " << std::get<PacketHeader>(*i).packetTimestamp << " " 
							<< std::get<PacketHeader>(positions[positionIndex]).packetTimestamp << " "
							<< std::get<PacketHeader>(attitudes[attitudeIndex]).packetTimestamp << std::endl;
					
					for(int lidarPoint = 0; lidarPoint < nbPoints; ++lidarPoint){
            			i++;
            		}
					
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
				
		        //Georeference::PoseidonFrameToEcef(georeferencedLaserPoint, *interpolatedAttitude, *interpolatedPosition, std::get<LidarPacket>(*i), leverArm, boresight);
		        
		        Georeference::PoseidonFrameToNed(georeferencedLaserPoint, ecefToNed, firstPosition, *interpolatedAttitude, *interpolatedPosition, std::get<LidarPacket>(*i), leverArm, boresight);
				
				
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

};

void printUsage(){
	std::cerr << "\n\
NAME\n\n\
	georeference - Produces a georeferenced point cloud from binary multibeam echosounder datagrams files\n\n\
SYNOPSIS\n \
	georeference [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] file\n\n\
DESCRIPTION\n \
	-L Use a local geographic frame (NED)\n \
	-T Use a terrestrial geographic frame (WGS84 ECEF)\n \
        -S choose one: nearestTime or nearestLocation\n\n \
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
		double roll     = 0.0;
		double pitch    = 0.0;
		double heading  = 0.0;


		int index;

		while((index=getopt(argc,argv,"x:y:z:r:p:h:s:S:LTg"))!=-1){
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
			}
		}
	
	//Lever arm
    Eigen::Vector3d leverArm;
    leverArm << leverArmX, leverArmY, leverArmZ;
    
    //Boresight
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight, roll, pitch, heading);
	
	PoseidonBinaryLidarGeoref georeferencer(fileName, leverArm, boresight);
	georeferencer.read();
	georeferencer.georeference();
	}
	
	return 0;
}
