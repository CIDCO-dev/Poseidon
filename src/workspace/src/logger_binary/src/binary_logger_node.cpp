#include "BinaryLogger.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "logger_binary");

  BinaryLogger logger;
  logger.run();
}
