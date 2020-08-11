#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <ctime>
#include <iomanip>
#include "Structs.hpp"

class Writer
{
public:
	Writer(std::string & gnssFilePath, std::string & imuFilePath, std::string & sonarFilePath, std::string separator=";"){
		init(gnss, imu, sonar, bin);
	}

	~Writer(){}

    bool getSetupOK() {
        return setupOK;
    }

	bool writeGnss(Position pos){
                if (!outGnss) {
                        std::cerr << "Could not open GNSS file" << std::endl;
                        return false;
                }
		outGnss << pos.timeStamp
     			<< sep << pos.x
     			<< sep << pos.y
		     	<< sep << pos.z
     			<< std::endl;
		return true;
	}

        bool writeImu(Imu imu){
                if (!outImu) {
                        std::cerr << "Could not open IMU file" << std::endl;
                        return false;
                }
                outImu << imu.timeStamp
                        << sep << imu.w
                        << sep << imu.x
                        << sep << imu.y
                        << sep << imu.z
                        << std::endl;
		return true;
        }

        bool writeSonar(Sonar sonar){
                if (!outImu) {
                        std::cerr << "Could not open SONAR file" << std::endl;
                        return false;
                }
                outSonar << sonar.timeStamp
                        << sep << sonar.depth
                        << std::endl;
		return true;
        }

	bool writeBinGnss(Position &pos){
		if (!outGnss) {
			std::cerr << "Could not open GNSS file" << std::endl;
			return false;
		}
		outGnss.write((char *)&pos, sizeof(Position));
		outGnss.flush();
		return true;
	}

	bool writeBinImu(Imu &imu){
		if (!outImu) {
			std::cerr << "Could not open IMU file" << std::endl;
			return false;
		}
		outImu.write((char *)&imu, sizeof(Imu));
		return true;
	}

	bool writeBinSonar(Sonar &sonar){
		if (!outSonar) {
			std::cerr << "Could not open SONAR file" << std::endl;
			return false;
		}
		outSonar.write((char *)&sonar, sizeof(Sonar));
		outSonar.flush();
		return true;
	}

	bool init(std::string gnssFileName, std::string imuFileName, std::string sonarFileName, bool isBinary){
	  	if(isBinary == false){
			outGnss.open(gnssFileName.c_str());
		  	outImu.open(imuFileName.c_str());
		  	outSonar.open(sonarFileName.c_str());
		}else{
                        outGnss.open(gnssFileName.c_str(), std::ios::binary);
                        outImu.open(imuFileName.c_str(), std::ios::binary);
                        outSonar.open(sonarFileName.c_str(), std::ios::binary);
		}
		// sep = ";";

	  	if (!outGnss) {
	     		std::cerr << "Could not open file: " << gnssFileName << std::endl;
	     		return false;
	  	}
	  	if (!outImu) {
	     		std::cerr << "Could not open file: " << imuFileName << std::endl;
	     		return false;
	  	}
	  	if (!outSonar) {
	     		std::cerr << "Could not open file: " << sonarFileName << std::endl;
	     		return false;
	  	}

		if(isBinary == false){
			outGnss << std::fixed << std::setprecision(10);
		  	outImu << std::fixed << std::setprecision(10);
		  	outSonar << std::fixed << std::setprecision(10);

	  		outGnss << "TimeStamp"
	     			<< sep << "Longitude"
		   		<< sep << "Latitude"
		 		<< sep << "Ellopsoidal Height"
			  	<< std::endl;

		  	outImu << "TimeStamp"
     				<< sep << "Orientation X"
     				<< sep << "Orientation Y"
	     			<< sep << "Orientation Z"
		     		<< sep << "Orientation W"
     				<< std::endl;

		 	outSonar << "TimeStamp"
		     		<< sep << "Depth"
     				<< std::endl;
		}

        setupOK = true;

	  	return true;
	}



    bool setupOK;

	std::ofstream outGnss;
	std::ofstream outImu;
	std::ofstream outSonar;

	std::string sep;
};

class Reader{
public:
	Reader(std::string gnssFileName, std::string imuFileName, std::string sonarFileName){
		binGnssFile = gnssFileName;
		binImuFile = imuFileName;
		binSonarFile = sonarFileName;

		std::string sep(".");
	 	outTextGnssFile = gnssFileName.substr(0, gnssFileName.find(sep));
		outTextGnssFile += ".txt";

                outTextImuFile = imuFileName.substr(0, imuFileName.find(sep));
                outTextImuFile += ".txt";

                outTextSonarFile = sonarFileName.substr(0, sonarFileName.find(sep));
                outTextSonarFile += ".txt";

		writer = new Writer(outTextGnssFile, outTextImuFile, outTextSonarFile, false);
	}
	~Reader(){}

	bool readGnss(){
		Position pos;

		std::ifstream ifs;
		ifs.open(binGnssFile.c_str(), std::ios::binary);

		if (ifs.is_open() == false){
		  std::cerr << "Cannot open GNSS bin file " << binGnssFile << std::endl;
		}

		while(ifs.good()){
		  	ifs.read(reinterpret_cast<char*> (&pos), sizeof(Position));
			writer->writeGnss(pos);
  		}
 		ifs.close();

		return true;
	}

	bool readImu(){
		Imu imu;

                std::ifstream ifs;
		ifs.open(binImuFile.c_str(), std::ios::binary);

  		if (ifs.is_open() == false){
    			std::cerr << "Cannot open IMU bin file " << binImuFile << std::endl;
  		}

  		while(ifs.good()){
    			ifs.read(reinterpret_cast<char*> (&imu), sizeof(Imu));
                        writer->writeImu(imu);
  		}
  		ifs.close();

		return true;
	}

	bool readSonar(){
                Sonar sonar;

                std::ifstream ifs;
                ifs.open(binSonarFile.c_str(), std::ios::binary);

                if (ifs.is_open() == false){
                        std::cerr << "Cannot open SONAR bin file " << binSonarFile << std::endl;
                }

                while(ifs.good()){
                        ifs.read(reinterpret_cast<char*> (&sonar), sizeof(Sonar));
                        writer->writeSonar(sonar);
                }
                ifs.close();

		return true;
        }

private:
	std::string outTextGnssFile;
	std::string outTextImuFile;
	std::string outTextSonarFile;

	std::string binGnssFile;
	std::string binImuFile;
	std::string binSonarFile;

	Writer *writer;
};

std::string getStringDate(){

        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        // strftime(buffer,sizeof(buffer),"%Y_%m_%d",timeinfo);
        strftime(buffer,sizeof(buffer),"%Y.%m.%d_%H%M%S",timeinfo);
        std::string date(buffer);

        return date;
}

