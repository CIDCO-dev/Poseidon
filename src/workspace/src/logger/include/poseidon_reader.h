#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include "types.h"

class PoseidonBinaryReader{
	public:
		PoseidonBinaryReader(std::string & filePath){
			file.open(filePath, std::ios::out | std::ios::binary);
			if(!file) {
			  std::cerr << "Cannot open file! " << filePath << std::endl;
			  exit(1);
			}
		}
		virtual void processGnss(PositionPacket & packet){
		}
		
		virtual void processImu(AttitudePacket & packet){
			std::cout<<"ok\n";
		}
		
		virtual void processSonar(DepthPacket & packet){
		}
		
		virtual void processLidar(LidarPacket & packet){
		}
		
		void read(){
		
			PacketHeader hdr;

			while(file.read((char *) &hdr, sizeof(PacketHeader))){
				if(hdr.packetType == 1){
					std::cout<<"gnssCallback \n";
					PositionPacket packet;
					file.read((char *) &packet, sizeof(PositionPacket));
					processGnss(packet);
				}
				
				else if(hdr.packetType == 2){
					std::cout<<"imuCallback \n";
					AttitudePacket packet;
					file.read((char *) &packet, sizeof(AttitudePacket));
					processImu(packet);
				}
				
				else if(hdr.packetType == 3){
					std::cout<<"sonarCallback \n";
					DepthPacket packet;
					file.read((char *) &packet, sizeof(DepthPacket));
					processSonar(packet);
				}
				
				else if(hdr.packetType == 4){
					std::cout<<"lidarCallback \n";
					int nbPoints = hdr.packetSize / sizeof(LidarPacket);
					for(int i =0; i<nbPoints; ++i){
						LidarPacket packet;
						file.read((char *) &packet, sizeof(LidarPacket));
						processLidar(packet);
					}
					
				}
				
				else{
					fprintf(stderr, "error, invalid packet type : %d", hdr.packetType);
					std::cerr<<std::endl;
				}
				
			}
			file.close();
		}
	
	private:
		std::ifstream file;
};
