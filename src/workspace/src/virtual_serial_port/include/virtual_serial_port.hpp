#ifndef virtual_serial_port
#define virtual_serial_port


class virtualSerialPort{
	public:
		virtualSerialPort(std::string Slave, std::string Master){
			this->slave = Slave;
			this->master = Master;
		}
		~virtualSerialPort(){}
		
		
		boost::process::child init(){
		/*
		socat should be installed -> sudo apt install socat
		user should be in dialout group
		once socat command launched : cat SLAVE
		echo test > MASTER
		*/
			//simpliest command
			//socat -d -d pty,raw,echo=0 pty,raw,echo=0
			std::string cmd = "socat -d -d pty,raw,echo=0,link="+ this->slave + " pty,raw,echo=0,link=" + this->master;
			boost::process::child virtDev(cmd); //TODO , handle errors
			std::this_thread::sleep_for(std::chrono::seconds(1));
			return virtDev;
		}
		
		void write(std::string message){
			//TODO , handle errors
			std::ofstream outfile (this->master, std::ofstream::binary);
			if(outfile.is_open()){
				outfile << message << "\r\n";
			}
			else{
				std::cerr<<"cannot open master";
			}
		}
		
		void close(boost::process::child &virtDev){
			virtDev.terminate();
			//delete master and slave file ?
		}
		
		
	private:
		std::string slave;
		std::string master;
};

#endif
