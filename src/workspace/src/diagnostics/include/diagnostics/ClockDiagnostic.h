#ifndef CLOCKDIAGNOISTIC_H
#define CLOCKDIAGNOISTIC_H

#include "DiagnosticsTest.h"
#include <libgpsmm.h>
#include <iostream>
#include <chrono>


class ClockDiagnostic : public DiagnosticsTest{
	
public:
	ClockDiagnostic(std::string name): DiagnosticsTest(name){}
	~ClockDiagnostic(){}

	void do_test()override{
		
		this->status = false;
		this->value = "";
		
		ros::Time startTime = ros::Time::now();
		double elapsedTime = 0.0;
		
		gpsmm gps_rec("localhost", "2947");

		if (gps_rec.stream(WATCH_ENABLE | WATCH_NEWSTYLE) == NULL) {
			this->value = "No GPSD running";
			this->status = false;
			return;
		}

		for (;;) {
			
			struct gps_data_t* gpsData = gps_rec.read();
			if (gpsData != nullptr) {
				if (gpsData->set & TIME_SET){
					//cout<<"gps status :" << gpsData->status <<"\n";
					struct timespec ts = gpsData->fix.time;
					auto systemTime = std::chrono::system_clock::now();
					auto systemTimePoint = std::chrono::time_point_cast<std::chrono::milliseconds>(systemTime);
					auto systemTimeMs = systemTimePoint.time_since_epoch().count();
					
					unsigned int timeDifferenceMs = std::abs(systemTimeMs-(ts.tv_sec * 1000 + ts.tv_nsec / 1000000));
					
					if(timeDifferenceMs > 1){
						this->value = "latency = " + std::to_string(timeDifferenceMs)+" milliseconds";
						this->status = false;
						break;
					}
					else{
						this->value = "latency = " + std::to_string(timeDifferenceMs)+" milliseconds";
						this->status = true;
					}
				}
			
				if (elapsedTime >= 2.0){
					break;
				}
			}
		}
	}
};
#endif
