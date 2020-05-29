#include "ros/ros.h"
#include "logger_binary/logger_binary.h"

Writer *writer;

int main(int argc, char **argv)
{

  if(argc < 2){

    std::cout << "nlogger_text, Missing output folder path" << std::endl;
    return 1;
  }


  std::string outputFolder( argv[1] );

  std::string outputGnssFile = outputFolder + "/" 
                                + getStringDate() + "_gnss.bin";
  std::string outputImuFile = outputFolder + + "/" 
                                + getStringDate() + "_imu.bin";
  std::string outputSonarFile = outputFolder + "/" 
                                + getStringDate() + "_sonar.bin";

  writer = new Writer(outputGnssFile, outputImuFile, outputSonarFile, false);

  if ( writer->getSetupOK() == false ) {
    std::cout << "logger_text, could not setup the writer" << std::endl;
    return 1;
  }
  ros::init(argc, argv, "logger_text");

  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("fix", 1000, gnssCallback);
  ros::Subscriber sub2 = n.subscribe("pose", 1000, imuCallback);
  ros::Subscriber sub3 = n.subscribe("depth", 1000, sonarCallback);

  ros::spin();

  return 0;
}
