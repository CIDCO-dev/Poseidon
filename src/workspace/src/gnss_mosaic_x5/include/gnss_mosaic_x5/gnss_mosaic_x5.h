#ifndef gnss_mosaic_x5
#define gnss_mosaic_x5

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <fcntl.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>

#include <chrono>
#include <cstdint>

#pragma pack(1)
typedef struct {
	uint16_t sync;
	uint16_t crc;
	uint16_t block_id;
	uint16_t length; // Total number of bytes in block including this header
} SbfBlockHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint64_t timeOfWeek; // milliseconds of current GPS week
	uint16_t weekNumber; // GPS week number
} SbfBlockTimeStamp;
#pragma pack()

class MosaicX5 {

public:
    MosaicX5(std::string & outputFolder, std::string & serialport): outputFolder(outputFolder), serialport(serialport){

    }

    void gnssCallback(const sensor_msgs::NavSatFix& fix){
        if(!bootstrappedGnssTime && fix.status.status >= 0){
            bootstrappedGnssTime = true;
        }
    }

    virtual void processDatagram(SbfBlockHeader & hdr, unsigned char * packet) {

	}

	void run() {
	    gnssSubscriber = n.subscribe("fix", 1000, &MosaicX5::gnssCallback,this);

	    //Wait for GNSS fix to init system time
        //bootstrappedGnssTime = true; // for now assume there is fix
        while(!bootstrappedGnssTime){
            sleep(1);
            ros::spinOnce();
        }

	    openAndInitSerialPort();

	    if(serial_port < 0) {
	        ROS_ERROR_STREAM("Can't run Mosaic-X5.");
	        exit(1);
	    }

	    // align serial port with Mosaic-X5 datagrams
        // do this for 2 seconds to clear buffer
	    auto time_start = std::chrono::system_clock::now();
        while( (std::chrono::system_clock::now() - time_start) < std::chrono::seconds{2} ) {
            readUntilAlignedWithDatagrams();
        }

        //create and open binary output file
        std::ofstream file;
        std::string outputFilename = outputFolder + std::string(datetime()) + std::string(".sbf");
        file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);

        if(!file) {
            ROS_ERROR_STREAM("Cannot open .sbf file on path: " << outputFilename);
            exit(1);
        } else {

            while(ros::ok()){
                //read a datagram from serial port and save in the file
                readAndProcessOneDatagram(serial_port, file);
            }

            close(serial_port);
            file.close();
        }

	}

protected:
    std::string outputFolder;
    std::string serialport;

	int serial_port = -1;

	bool bootstrappedGnssTime = false;
	ros::NodeHandle n;
	ros::Subscriber gnssSubscriber;

	bool aligned = false;

	void readAndProcessOneDatagram(int serial_port, std::ofstream & file) {
	    SbfBlockHeader hdr;
	    read(serial_port, &hdr, sizeof(SbfBlockHeader));

	    uint16_t blockPayloadLength = hdr.length - 8; // header is 8 bytes
	    unsigned char * payload = (unsigned char*) malloc(blockPayloadLength);
	    read(serial_port, payload, blockPayloadLength);

	    processDatagram(hdr, payload);

	    //write header
	    file.write((char*)&hdr, sizeof(SbfBlockHeader));
	    //write payload
	    file.write((char*)payload, blockPayloadLength);

	    // clean up
	    free(payload);
	}

	void readUntilAlignedWithDatagrams() {
		int size = 1;
		char read_buf [size];

		while (!aligned) {
			read(serial_port, &read_buf, size);


			if(read_buf[0] == 0x24) {
			    ROS_ERROR_STREAM("read_buf[0]: " << (int)read_buf[0]);
			    read(serial_port, &read_buf, size);

			    if(read_buf[0] == 0x40) {
			        uint16_t header_buf [3];
			        read(serial_port, &header_buf, 6);
			        uint16_t blockPayloadLength = header_buf[2] - 8;

			        char payload [blockPayloadLength];
				    read(serial_port, &payload, blockPayloadLength);


				    aligned = true;
			    }
			}
		}
	}

	void openAndInitSerialPort() {
        serial_port = open(serialport.c_str(), O_RDONLY | O_NOCTTY);

        if(serial_port < 0) {
		    ROS_ERROR_STREAM("Couldn't open serial port on path: " << serialport);
		    return;
		}

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if(tcgetattr(serial_port, &tty) != 0) {
            ROS_ERROR_STREAM("Error " << errno << " from tcgetattr: " << strerror(errno));
            return;
        }

        //tty flags copied from gnss_zed_f9p package

        tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag |= CS8; // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
            return;
        }
    }

    std::string datetime(){
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];

		time (&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer,80,"%Y.%m.%d_%H%M%S",timeinfo);
		return std::string(buffer);
	}
};

#endif