#include "poseidon_reader.h"
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <memory>

namespace Interpolator{
	
		double linearInterpolationByTime(double y1, double y2, uint64_t targetTimestamp, uint64_t t1, uint64_t t2) {
			if (t1 == t2){
				// throw new Exception("The two positions timestamp are the same");
			}
			if (t1 > targetTimestamp){
				// throw new Exception("The first position timestamp is higher than interpolation timestamp");
			}
			if (t1 > t2){
				// throw new Exception("The first position timestamp is higher than the second position timestamp");
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
			/*
			std::stringstream ss;        
			ss << "The angles " << a1 << " and " << a2
			<< " have a difference of 180 degrees which means there are two possible answers at timestamp " << targetTimestamp;
			throw new Exception(ss.str());
			*/
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
	
	// WGS84 ellipsoid Parameters

	/**WGS84 ellipsoid semi-major axis*/
	static constexpr double a = 6378137.0;

	/**WGS84 ellipsoid first eccentricity squared*/
	static constexpr double e2 = 0.081819190842622 * 0.081819190842622;

	/**WGS84 ellipsoid inverse flattening*/
	static constexpr double f = 1.0 / 298.257223563;

	/**WGS84 ellipsoid semi-minor axis*/
	static constexpr double b = a * (1-f); // semi-minor axis

	/**WGS84 ellipsoid second eccentricity squared*/
	static constexpr double epsilon = e2 / (1.0 - e2); // second eccentricity squared

	
	
	void matrixWgs84ToEcef(Eigen::Matrix3d & outputMatrix, PositionPacket & position){
		//outputMatrix << TODO
		
		return;
	}

	
	void PoseidonFrameToEcef(Eigen::Vector3d & pointsToGeoref, AttitudePacket & attitude, PositionPacket & position, 
						LidarPacket & point, Eigen::Vector3d & leverArm, Eigen::Matrix3d & boresight) {
	
		Eigen::Matrix3d wgs84ToEcef;
    	matrixWgs84ToEcef(wgs84ToEcef, position);
    	
		return;
	}
	
}


class PoseidonBinaryLidarGeoref : public PoseidonBinaryReader{
	public:
		PoseidonBinaryLidarGeoref(std::string & filePath) : PoseidonBinaryReader(filePath){}
		~PoseidonBinaryLidarGeoref(){}
		
		void processGnss(PacketHeader & hdr, PositionPacket & packet)override{
			std::pair pairPacket = std::make_pair(hdr, packet);
			positions->push_back(pairPacket);
		}
		
		void processImu(PacketHeader & hdr, AttitudePacket & packet)override{
			std::pair pairPacket = std::make_pair(hdr, packet);
			attitudes->push_back(pairPacket);
		}
		
		void processLidar(PacketHeader & hdr, LidarPacket &packet)override{
			std::pair pairPacket = std::make_pair(hdr, packet);
			laserPoints->push_back(pairPacket);
		}
		
		std::vector<std::pair<PacketHeader, PositionPacket>> getPositions(){
			return *positions;
		}
		
		
		std::vector<std::pair<PacketHeader, AttitudePacket>> getAtitudes(){
			return *attitudes;
		}
		
		
		std::vector<std::pair<PacketHeader, LidarPacket>> getLaserPoints(){
			return *laserPoints;
		}
		
		
		void georeference(){
			interpolate();
			
		}
		
		template <typename T>
		static bool sortByTimestamp(std::pair<PacketHeader, T> & pair1, std::pair<PacketHeader, T> & pair2){
			PacketHeader header1 = std::get<PacketHeader>(pair1);
			PacketHeader header2 = std::get<PacketHeader>(pair2);
	
			return header1.packetTimestamp < header2.packetTimestamp;
		}
		
		
	private:
		
		std::vector<std::pair<PacketHeader, PositionPacket>> *positions = new std::vector<std::pair<PacketHeader, PositionPacket>> ;
		std::vector<std::pair<PacketHeader, AttitudePacket>> *attitudes = new std::vector<std::pair<PacketHeader, AttitudePacket>>;
		std::vector<std::pair<PacketHeader, LidarPacket>>  *laserPoints = new std::vector<std::pair<PacketHeader, LidarPacket>>;


		void interpolate(){
			if(this->positions->size()==0){
				std::cerr << "[-] No position data found in file" << std::endl;
				return;
			}

		    if(this->attitudes->size()==0){
	            std::cerr << "[-] No attitude data found in file" << std::endl;
	            return;
		    }

		    if(this->laserPoints->size()==0){
	            std::cerr << "[-] No laserPoints data found in file" << std::endl;
	            return;
		    }
		    
			// TODO Compute centroid or use first position
			
			//Sort everything
		    std::sort(positions->begin(), positions->end(), &sortByTimestamp<PositionPacket>);
		    std::sort(attitudes->begin(), attitudes->end(), &sortByTimestamp<AttitudePacket>);
		    std::sort(laserPoints->begin(), laserPoints->end(), &sortByTimestamp<LidarPacket>);
			
			// For correct display of timestamps on Windows
		    std::cerr <<  "[+] Position data points: " << positions->size() << " [" << std::get<PacketHeader>(positions->at(0)).packetTimestamp << " to " 
		            << std::get<PacketHeader>(positions->at(positions->size() - 1)).packetTimestamp << "]\n";
		            
		    std::cerr <<  "[+] Attitude data points: " << attitudes->size() << " [" << std::get<PacketHeader>(attitudes->at(0)).packetTimestamp << " to " 
		            << std::get<PacketHeader>(attitudes->at(attitudes->size() - 1)).packetTimestamp << "]\n";  
		                
		    std::cerr <<  "[+] Ping data points: " << laserPoints->size() << " [" 
					<< ( (laserPoints->size() > 0) ? std::get<PacketHeader>(laserPoints->at(0)).packetTimestamp : 0 ) << " to " 
		            << ( (laserPoints->size() > 0) ? std::get<PacketHeader>(laserPoints->at(laserPoints->size() - 1)).packetTimestamp : 0 ) << "]\n";
			
			
			//interpolate attitudes and positions around pings
        	unsigned int attitudeIndex = 0;
        	unsigned int positionIndex = 0;
        	
        	//Georef pings
        	for (auto i = laserPoints->begin(); i != laserPoints->end(); i++) {
        		
        		while (attitudeIndex + 1 < attitudes->size() && 
						std::get<PacketHeader>(attitudes->at(attitudeIndex + 1)).packetTimestamp < std::get<PacketHeader>(*i).packetTimestamp) 
				{	
		            attitudeIndex++;
		        }

		        //No more attitudes available
		        if (attitudeIndex >= attitudes->size() - 1) {
		            //std::cerr << "No more attitudes" << std::endl;
		            break;
		        }

		        while (positionIndex + 1 < positions->size() && 
						std::get<PacketHeader>(positions->at(positionIndex + 1)).packetTimestamp < std::get<PacketHeader>(*i).packetTimestamp) 
				{
		            positionIndex++;
		        }
		        
		        //No more positions available
		        if (positionIndex >= positions->size() - 1) {
		            //std::cerr << "No more positions" << std::endl;
		            break;
		        }
		        
		        //No position or attitude smaller than ping, so discard this ping
            	if (std::get<PacketHeader>(positions->at(positionIndex)).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp 
					|| std::get<PacketHeader>(attitudes->at(attitudeIndex)).packetTimestamp > std::get<PacketHeader>(*i).packetTimestamp
				) {
            		
            		int nbPoints = std::get<PacketHeader>(*i).packetSize / sizeof(LidarPacket);
            		
            		std::cerr << "rejecting " << nbPoints << " lidar point " << std::get<PacketHeader>(*i).packetTimestamp << " " 
							<< std::get<PacketHeader>(positions->at(positionIndex)).packetTimestamp << " "
							<< std::get<PacketHeader>(attitudes->at(attitudeIndex)).packetTimestamp << std::endl;
					
					for(int lidarPoint = 0; lidarPoint < nbPoints; ++lidarPoint){
            			i++;
            		}
					
		            continue;
		            
		        }
		    
			AttitudePacket * interpolatedAttitude = Interpolator::interpolateAttitude(attitudes->at(attitudeIndex), 
																				  attitudes->at(attitudeIndex + 1), 
																				  std::get<PacketHeader>(*i).packetTimestamp);
	        
	        
	        PositionPacket * interpolatedPosition = Interpolator::interpolatePosition(positions->at(positionIndex), 
																					  positions->at(positionIndex + 1),
												 									  std::get<PacketHeader>(*i).packetTimestamp);
            
			
            //georeference
            Eigen::Vector3d georeferencedPing;
			/*
            georef.georeference(georeferencedPing, *interpolatedAttitude, *interpolatedPosition, (*i), leverArm, boresight);

            processGeoreferencedPing(georeferencedPing, (*i).getQuality(), (*i).getIntensity(), positionIndex, attitudeIndex);
			*/
			
            delete interpolatedAttitude;
            delete interpolatedPosition;
		    	
		    }
			
		}
};



int main(int argc,char** argv){

	if(argc != 2){
		std::cerr << "poseidonReader filePath" << std::endl;
		exit(1);
	}
	std::string filePath = argv[1];
	PoseidonBinaryLidarGeoref georeferencer(filePath);
	georeferencer.read();
	georeferencer.georeference();
	
	return 0;
}
