#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include "types.h"

class PoseidonBinaryReader{
	public:
		PoseidonBinaryReader(std::string filePath){
			file.open(filePath, std::ios::out | std::ios::binary);
			if(!file) {
			  std::cout << "Cannot open file!" << std::endl;
			  exit(1);
			}
		}
		
		void processGnss(PacketHeader & hdr, PositionPacket & packet){}
		
		void processImu(PacketHeader & hdr, AttitudePacket & packet){}
		
		void processSonar(PacketHeader & hdr, DepthPacket & packet){}
		
		void processLidar(PacketHeader & hdr, LidarPacket & packet){
			//printf("x %f , y %f , z %f\n", packet.laser_x, packet.laser_y, packet.laser_z);
		}
		
		void processHeader(PacketHeader & hdr){
		
			if(hdr.packetType == 1){
				std::cout<<"gnssCallback \n";
				PositionPacket packet;
				file.read((char *) &packet, sizeof(PositionPacket));
				processGnss(hdr, packet);
			}
			
			else if(hdr.packetType == 2){
				std::cout<<"imuCallback \n";
				AttitudePacket packet;
				file.read((char *) &packet, sizeof(AttitudePacket));
				processImu(hdr, packet);
			}
			
			else if(hdr.packetType == 3){
				std::cout<<"sonarCallback \n";
				DepthPacket packet;
				file.read((char *) &packet, sizeof(DepthPacket));
				processSonar(hdr, packet);
			}
			
			else if(hdr.packetType == 4){
				std::cout<<"lidarCallback \n";
				LidarPacket packet;
				file.read((char *) &packet, sizeof(LidarPacket) * hdr.packetSize);
				processLidar(hdr, packet);
			}
			
			else{
				fprintf(stderr, "error, invalid packet type : %d", hdr.packetType);
				std::cerr<<std::endl;
			}
		}
		void read(){
			
				PacketHeader hdr;

				while(file.read((char *) &hdr, sizeof(PacketHeader))){
					if(hdr.packetType == 1 || hdr.packetType == 2 || hdr.packetType == 3 || hdr.packetType == 4){
						processHeader(hdr);
					}
				}
				
			
		
			
		}
	
	private:
		std::ifstream file;
};

int main(int argc,char** argv){

	if(argc != 2){
		std::cerr << "poseidonReader filePath" << std::endl;
		exit(1);
	}
	
	PoseidonBinaryReader reader(argv[1]);
	reader.read();
	
	return 0;
}
