#include <string>
#include <iostream>
#include "../../utils/ReaderWriter.hpp"

int main(int argc, char **argv){

  if (argc != 4){
    std::cout << "Usages: binary_decoder [gnss.bin] [imu.bin] [sonar.bin]" << std::endl;
    return 1;
  }

  std::string gnss(argv[1]);
  std::string imu(argv[2]);
  std::string sonar(argv[3]);

  Reader *reader = new Reader(gnss, imu, sonar);
  reader->readGnss();
  reader->readImu();
  reader->readSonar();

  return 0;
}
