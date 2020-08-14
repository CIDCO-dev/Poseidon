#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <ctime>

class TimeUtils{
public:

	static uint64_t buildTimeStamp(int sec, int nsec){
		uint64_t timestamp;
		timestamp = sec;
		timestamp = (timestamp * 1000000000)+nsec;
		return timestamp;
	}

	static std::string getStringDate(){
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
};

#endif
