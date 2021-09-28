#ifndef mosaicx5datagramcounter
#define mosaicx5datagramcounter

#include <gnss_mosaic_x5/gnss_mosaic_x5.h>
#include <map>
#include <bitset>

class MosaicX5DatagramCount : MosaicX5 {

public:
    //Store datagram counts for each type
    std::map<uint16_t, int> datagramCount;

    //default run for 1 minute and count datagrams
    unsigned int minutes = 60;

    MosaicX5DatagramCount(std::string & outputFolder, std::string & serialport): MosaicX5(outputFolder, serialport) {

    }

    void run() {
        openAndInitSerialPort();

	    if(serial_port < 0) {
	        ROS_ERROR_STREAM("Can't run Mosaic-X5.");
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
        std::string outputFilename = outputFolder + std::string(datetime()) + std::string(".sbf");
        file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);

        if(!file) {
            ROS_ERROR_STREAM("Cannot open .sbf file on path: " << outputFilename);
            exit(1);
        } else {
            ROS_ERROR_STREAM("Open for buisness");
            // align serial port with piksi datagrams
            readUntilAlignedWithDatagrams();

            ROS_ERROR_STREAM("Aligned, reading for " << minutes << " minutes");

            auto time_start = std::chrono::system_clock::now();
            while( (std::chrono::system_clock::now() - time_start) < std::chrono::minutes(minutes) ) {
                readAndProcessOneDatagram(serial_port, file);
            }

            ROS_ERROR_STREAM("Done, closing up shop.");

            close(serial_port);
            file.close();
        }
    }

    void processDatagram(SbfBlockHeader & hdr, unsigned char * packet) {
        //Count datagrams

        //uint16_t block_id = hdr.block_id;

        //std::bitset<16> bset(block_id);

        //ROS_ERROR_STREAM("block id: " << bset);

	    if(datagramCount.find(hdr.block_id) == datagramCount.end()) {
		    datagramCount[hdr.block_id] = 1;
		} else {
		    datagramCount[hdr.block_id] = datagramCount[hdr.block_id] + 1;
		}
	}

};

#endif