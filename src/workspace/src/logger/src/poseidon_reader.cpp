#include "poseidon_reader.h"

int main(int argc,char** argv){

	if(argc != 2){
		std::cerr << "poseidonReader filePath" << std::endl;
		exit(1);
	}
	std::string filePath = argv[1];
	PoseidonBinaryReader reader(filePath);
	reader.read();
	
	return 0;
}
