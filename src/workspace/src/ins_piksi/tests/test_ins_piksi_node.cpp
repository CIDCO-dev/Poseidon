#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>

#include <bitset>
#include <thread>

#include <fstream>
#include <fcntl.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>

#pragma pack(1)
typedef struct {
	uint8_t preamble;
	uint16_t message_type;
	uint16_t sender_id;
	uint8_t length;
} SbpHeader;
#pragma pack()

class Piksi {
public:
	virtual void run()=0;
    
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

class PiksiOneDatagramAtATime : Piksi {

private:
	std::string outputFolder;
	std::string serialport;

	//std_msgs::String result;
	//std_msgs::String ubx_msg_str;
	//std_msgs::String ubx_read;

	bool bootstrappedGnssTime = false;

	//ros::NodeHandle n;

	//ros::Subscriber gnssSubscriber;

public:

	PiksiOneDatagramAtATime(std::string & outputFolder, std::string & serialport): outputFolder(outputFolder), serialport(serialport){
		//gnssSubscriber = n.subscribe("fix", 1000, &PiksiOneByteAtATime::gnssCallback,this);
	}
	
	/*
	void gnssCallback(const sensor_msgs::NavSatFix& fix){
		if(!bootstrappedGnssTime && fix.status.status >= 0){
			bootstrappedGnssTime = true;
		}
	}
	*/
	
	void processDatagram(SbpHeader & hdr, unsigned char * packet) {
		/*
		if(hdr.message_type == THE_PACKET_TYPE) {
			
			// TODO: implement
		}
		*/
		
		
	
	}

	void run() {
		int serial_port = open(serialport.c_str(), O_RDONLY | O_NOCTTY);

		if(serial_port < 0) {
		    ROS_ERROR_STREAM("Couldn't open serial port on: " << serialport);
		    close(serial_port);
		} else {
			struct termios tty;
			memset(&tty, 0, sizeof tty);

			if(tcgetattr(serial_port, &tty) != 0) {
			    ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
			    close(serial_port);
			} else {

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

				// Set in/out baud rate to be 9600
				cfsetispeed(&tty, B460800);
				cfsetospeed(&tty, B460800);

			    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
				ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
				close(serial_port);
			    } else {
				ROS_ERROR_STREAM("Success!");

				

				//Wait for GNSS fix to init system time

				bootstrappedGnssTime = true; // for test assume there is fix
				while(!bootstrappedGnssTime){
				    sleep(1);
				    ros::spinOnce();
				}

				//create and open file
				std::ofstream file;
				std::string outputFilename = outputFolder + std::string(datetime()) + std::string("_OneDatagramAtATime.sbp");
				file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);
				if(file) {
				
				    //
				
				    // read data from serial port into datagram struct
				    //Init buffer
				    
				    
				    // TODO: implement function to identify a first 0x55 that identifies a datagram
				    
				    // TODO: implement function to loop over following datagrams and write in file until ros::ok() returns false
				    
				    
				    
				    
				    
				} else {
				    ROS_ERROR_STREAM("Couldn't sbp file: " << outputFilename);
				}
				
				

				//cleanup
				close(serial_port);
				file.close();
			    }
			}
		}
	
	}
                
                

};

class PiksiOneByteAtATime : Piksi {

private:
	 	std::string outputFolder;
		std::string serialport;

		//std_msgs::String result;
		//std_msgs::String ubx_msg_str;
		//std_msgs::String ubx_read;

		bool bootstrappedGnssTime = false;

                //ros::NodeHandle n;

                //ros::Subscriber gnssSubscriber;

public:

	PiksiOneByteAtATime(std::string & outputFolder, std::string & serialport): outputFolder(outputFolder), serialport(serialport){
		//gnssSubscriber = n.subscribe("fix", 1000, &PiksiOneByteAtATime::gnssCallback,this);
	}
	
	/*
	void gnssCallback(const sensor_msgs::NavSatFix& fix){
		if(!bootstrappedGnssTime && fix.status.status >= 0){
			bootstrappedGnssTime = true;
		}
	}
	*/
	
	

	void run() {
		int serial_port = open(serialport.c_str(), O_RDONLY | O_NOCTTY);

		if(serial_port < 0) {
		    ROS_ERROR_STREAM("Couldn't open serial port on: " << serialport);
		    close(serial_port);
		} else {
			struct termios tty;
			memset(&tty, 0, sizeof tty);

			if(tcgetattr(serial_port, &tty) != 0) {
			    ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
			    close(serial_port);
			} else {

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

				// Set in/out baud rate to be 9600
				cfsetispeed(&tty, B460800);
				cfsetospeed(&tty, B460800);

			    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
				ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
				close(serial_port);
			    } else {
				ROS_ERROR_STREAM("Success!");

				//Init buffer
				int size = 1;
				char read_buf [size];

				//Wait for GNSS fix to init system time

				bootstrappedGnssTime = true; // for test assume there is fix
				while(!bootstrappedGnssTime){
				    sleep(1);
				    ros::spinOnce();
				}

				//create and open file
				std::ofstream file;
				std::string outputFilename = outputFolder + std::string(datetime()) + std::string("_oneByteAtATime.sbp");
				file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);
				if(file) {

				    int numBytes = 2<<10; // 1 kilobyte
				    int count = 0;
				    while(count < numBytes) { //read serial port and save in the file
					    read(serial_port, &read_buf, size);
					    file.write(read_buf, size);
					    count++;

					    if( count == 2<<8 ||
						count == 2<<9 ||
						count == 2<<10 ||
						count == 2<<11 ||
						count == 2<<12 ||
						count == 2<<13 ||
						count == 2<<14 ||
						count == 2<<15 ||
						count == 2<<16 ||
						count == 2<<17 ||
						count == 2<<18 ||
						count == 2<<19) {
						ROS_ERROR_STREAM("Bytes written = " << count);
					    }
				    }

				//cleanup
				    close(serial_port);
				    file.close();

				} else {
				    ROS_ERROR_STREAM("Couldn't sbp file: " << outputFilename);
				}
			    }
			}
		}
	
	}

};

TEST(PiksiTestSuite, testCaseCanReceiveDataLikeZedF9PClass) {

	std::string serialPortPath = "/dev/ttyACM0";
	std::string logPath = "";

	PiksiOneByteAtATime piksi(logPath, serialPortPath);
	piksi.run();
}


TEST(PiksiTestSuite, testCaseCanReceiveDataLikeZedF9PClassless) {

    std::string serialPortPath = "/dev/ttyACM0";

	int serial_port = open(serialPortPath.c_str(), O_RDONLY | O_NOCTTY);

	if(serial_port < 0) {
	    ROS_ERROR_STREAM("Couldn't open serial port on: " << serialPortPath);
	    close(serial_port);
	} else {
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if(tcgetattr(serial_port, &tty) != 0) {
            ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
            close(serial_port);
        } else {

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

			// Set in/out baud rate to be 9600
			cfsetispeed(&tty, B460800);
			cfsetospeed(&tty, B460800);

            if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
                close(serial_port);
            } else {
                ROS_ERROR_STREAM("Success!");

                //Init buffer
                int size = 1;
                char read_buf [size];

                //Wait for GNSS fix to init system time

                bool bootstrappedGnssTime = true; // for test assume there is fix
                while(!bootstrappedGnssTime){
                    sleep(1);
                    ros::spinOnce();
                }

                //create and open file
                std::ofstream file;
                std::string outputFilename = "/home/renaud/test.sbp";
                file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);
                if(file) {

                    int numBytes = 2<<10; // 1 kilobyte
                    int count = 0;
                    while(count < numBytes) { //read serial port and save in the file
                        int n = read(serial_port, &read_buf, size);
					    file.write(read_buf, size);
					    count++;

					    if( count == 2<<8 ||
					        count == 2<<9 ||
					        count == 2<<10 ||
					        count == 2<<11 ||
					        count == 2<<12 ||
					        count == 2<<13 ||
					        count == 2<<14 ||
					        count == 2<<15 ||
					        count == 2<<16 ||
					        count == 2<<17 ||
					        count == 2<<18 ||
					        count == 2<<19) {
					        ROS_ERROR_STREAM("Bytes written = " << count);
					    }
                    }

		//cleanup
                    close(serial_port);
                    file.close();

                } else {
                    ROS_ERROR_STREAM("Couldn't sbp file: " << outputFilename);
                }
            }
        }
	}

	//close(serial_port);
}

/*
TEST(PiksiTestSuite, testCaseCanReceiveData) {

    int howManyDatagrams = 1000;

    //std::string logPath (argv[1]);
	std::string serialPortPath = "/dev/ttyACM0";

	int serial_port = open(serialPortPath.c_str(), O_RDONLY | O_NOCTTY);

	if(serial_port < 0) {
	    ROS_ERROR_STREAM("Couldn't open serial port on: " << serialPortPath);
	} else {
        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if(tcgetattr(serial_port, &tty) != 0) {
            ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
        } else {

        }

	}

    tty.c_cflag = CRTSCTS | CS8 | CLOCAL | CREAD;
    tty.c_lflag = 0;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_cc[VTIME] = 0; // no inter-character timer
    tty.c_cc[VMIN] = 5; // blocking read until 1 char is received

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
        //printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    char find_datagram_buf;

    bool lookingForFirstDatagram = true;
    int sizeCRC = 2;
    while(lookingForFirstDatagram) { //read serial port until found 0x55
        int n = read(serial_port, &find_datagram_buf, 1);
        if(find_datagram_buf == 0x55) {
            std::bitset<8> x(find_datagram_buf);
            ROS_ERROR_STREAM("Found first datagram: " << x);


            int sizePreambleFollowup = 5;  // 5 = 6-1, we already have th 0x55
            char first_datagram_preamble[sizePreambleFollowup];
            read(serial_port, &first_datagram_preamble, sizePreambleFollowup);
            if(false) {

            }



            int sizePayload = first_datagram_preamble[4];
            if(sizePayload < 0) {
                continue;
            }

            char first_datagram_payloadBuffer[sizePayload];
            read(serial_port, &first_datagram_payloadBuffer, sizePayload);

            char first_datagram_CRC[sizeCRC];
            read(serial_port, &first_datagram_CRC, sizeCRC);

            //lookingForFirstDatagram = false;
            ROS_ERROR_STREAM("Sender: " << (int)first_datagram_preamble[2] << " " << (int)first_datagram_preamble[3]);
            ROS_ERROR_STREAM("Length of first datagram payload: " << sizePayload << " bytes.");
        } else {
            usleep(1000);
        }
    }

    std::ofstream file;
    std::string outputFilename = "test.sbp";
    file.open(outputFilename.c_str(),std::ios::app|std::ios::out|std::ios::binary);

    if(file) {
        int count = 0;
        while(count < howManyDatagrams) {


        }
    } else {
        ROS_ERROR_STREAM("Could not open file: " << outputFilename);
    }

    file.close();
	close(serial_port);

    ros::NodeHandle nh;

    ROS_ERROR_STREAM("testing 1 2 3" << serialPortPath );

    //verify that callback was called by subscriber
    ASSERT_TRUE(true) << "callback was not called by subscriber";
}
*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestInertialsenseNode");

    testing::InitGoogleTest(&argc, argv);

    std::thread t([]{ros::spin();}); // let ros spin in its own thread

    auto res = RUN_ALL_TESTS();

    ros::shutdown(); // this will cause the ros::spin() to return
    t.join();

    return res;
}
