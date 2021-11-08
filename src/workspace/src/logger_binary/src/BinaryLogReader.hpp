#pragma once

#include <fstream>
#include <string>
#include "types.h"

class BinaryLogReader{
public:
	BinaryLogReader(std::string & fileName) : fileName(fileName){

	}

	void parse(){
		ifstream inputFile;

		inputFile.open(fileName,std::ios::binary);

		if(inputFile.is_open()){
			...
		}
		else{
			throw invalid_argument("Cannot open file " + fileName);
		}
	}
private:
	std::string fileName;
};


