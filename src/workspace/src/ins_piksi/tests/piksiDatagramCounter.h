#ifndef piksidatagramcounter
#define piksidatagramcounter

#include <ins_piksi/ins_piksi.h>

#include <map>

class PiksiDatagramCount : Piksi {
public:
    //Store datagram counts for each type
    std::map<uint16_t, int> datagramCount;

    //default run for 1 minute and count datagrams
    unsigned int minutes = 1;

    PiksiDatagramCount(std::string & outputFolder, std::string & serialport): Piksi(outputFolder, serialport) {

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
            while( (std::chrono::system_clock::now() - time_start) < std::chrono::minutes(minutes) ) {
                readAndProcessOneDatagram(serial_port, file);
            }

            close(serial_port);
            file.close();
        }
    }

    void processDatagram(SbpHeader & hdr, unsigned char * packet) {
        //Count datagrams
	    if(datagramCount.find(hdr.message_type) == datagramCount.end()) {
		    datagramCount[hdr.message_type] = 1;
		} else {
		    datagramCount[hdr.message_type] = datagramCount[hdr.message_type] + 1;
		}
	}
};

#endif