#include "poseidon_reader.h"
#include <SbetProcessor.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "georeferencing_utils.h"

class VitalsPrinter : public PoseidonBinaryReader{
	public:
		VitalsPrinter(std::string & filePath) : PoseidonBinaryReader(filePath){}
		~VitalsPrinter(){}
		
		void processVital(PacketHeader & hdr, VitalPacket & packet) override{
			std::cout<< packet.valueName << " : " << packet.value << "\n";
		}
		
		void processSpeed(PacketHeader & hdr, SpeedPacket & packet) override{
			std::cout<< "speed : " << packet.speedKMH << "\n";
		}
};


int main(int argc,char** argv){

	if(argc != 2){
		std::cerr << "vital_printer logFile" << std::endl;
		exit(1);
	}
	std::string poseidonFilePath = argv[1];
	
	VitalsPrinter printer(poseidonFilePath);
	printer.read();
	
	return 0;
}
