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
		virtual void processGnss(PacketHeader & hdr, PositionPacket & packet){}
		virtual void processImu(PacketHeader & hdr, AttitudePacket & packet){}
		virtual void processSonar(PacketHeader & hdr, DepthPacket & packet){}
		virtual void processLidar(PacketHeader & hdr, LidarPacket & packet){}
		virtual void processVital(PacketHeader & hdr, VitalPacket & packet){}
		virtual void processSpeed(PacketHeader & hdr, SpeedPacket & packet){}
		
		void read(){
		
			PacketHeader hdr;

			while(file.read((char *) &hdr, sizeof(PacketHeader))){
				if(hdr.packetType == PACKET_POSITION){
					//std::cout<<"gnssCallback \n";
					PositionPacket packet;
					file.read((char *) &packet, sizeof(PositionPacket));
					processGnss(hdr, packet);
				}
				
				else if(hdr.packetType == PACKET_ATTITUDE){
					//std::cout<<"imuCallback \n";
					AttitudePacket packet;
					file.read((char *) &packet, sizeof(AttitudePacket));
					processImu(hdr, packet);
				}
				
				else if(hdr.packetType == PACKET_DEPTH){
					//std::cout<<"sonarCallback \n";
					DepthPacket packet;
					file.read((char *) &packet, sizeof(DepthPacket));
					processSonar(hdr, packet);
				}
				
				else if(hdr.packetType == PACKET_LIDAR){
					//std::cout<<"lidarCallback \n";
					int nbPoints = hdr.packetSize / sizeof(LidarPacket);
					for(int i =0; i<nbPoints; ++i){
						LidarPacket packet;
						file.read((char *) &packet, sizeof(LidarPacket));
						processLidar(hdr, packet);
					}
				}
				else if(hdr.packetType == PACKET_SPEED){
					SpeedPacket packet;
					file.read((char *) &packet, sizeof(SpeedPacket));
					processSpeed(hdr, packet);
				}
				else if(hdr.packetType == PACKET_VITALS){
					VitalsPacket packet;
					file.read((char *) &packet, sizeof(VitalsPacket));
					for(int i = 0; i<packet.nbValues; i++){
						VitalPacket vital;
						file.read((char *) &vital, sizeof(int));
						vital.valueName.resize(vital.valueNameSize);
						file.read((char *) &vital.valueName[0], (vital.valueNameSize * sizeof(char)));
						file.read((char *) &vital.value, sizeof(double));
						processVital(hdr, vital);
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
