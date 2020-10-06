#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <ctime>

class TimeUtils{
public:

	//returns timestamp in microseconds
	static uint64_t buildTimeStamp(int sec, int nsec){
		uint64_t timestamp;
		timestamp = sec;
		timestamp = (timestamp * 1000000)+nsec/1000;
		return timestamp;
	}

	static std::string getTimestampString(int sec,int nsec){
		time_t seconds = sec;
		struct tm * timeinfo;
		char buffer[80];

		timeinfo = localtime(&seconds);

                strftime(buffer,sizeof(buffer),"%Y-%m-%d %H:%M:%S",timeinfo);
                std::string date(buffer);

		memset(buffer,0,sizeof(buffer));
		sprintf(buffer,"%06d",nsec/1000);
		date = date + std::string(".") + std::string(buffer);

                return date;
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
