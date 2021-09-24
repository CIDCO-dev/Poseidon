#ifndef piksionehourlog
#define piksionehourlog

#include <ins_piksi/ins_piksi.h>

class PiksiOneHourLog : Piksi {
public:

    //default run for 60 minute and count datagrams
    unsigned int minutes = 60;

    PiksiOneHourLog(std::string & outputFolder, std::string & serialport): Piksi(outputFolder, serialport) {

    }

    void run() {
        openAndInitSerialPort();

	    if(serial_port < 0) {
	        ROS_ERROR_STREAM("Can't run Piksi.");
	        exit(1);
	    }

	    //Wait for GNSS fix to init system time
        bootstrappedGnssTime = true; // for now assume there is fix
        while(!bootstrappedGnssTime){
            sleep(1);
            ros::spinOnce();
        }

        //create and open binary output file
        std::ofstream file;
        std::string outputFilename = outputFolder + std::string(datetime()) + std::string(".sbp");
        file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);

        if(!file) {
            ROS_ERROR_STREAM("Cannot open .sbp file on path: " << outputFilename);
            exit(1);
        } else {
            // align serial port with piksi datagrams
            readUntilAlignedWithDatagrams();

            auto time_start = std::chrono::system_clock::now();
            while( (std::chrono::system_clock::now() - time_start) < std::chrono::minutes{minutes} ) {
                readAndProcessOneDatagram(serial_port, file);
            }

            close(serial_port);
            file.close();
        }
    }
};

#endif