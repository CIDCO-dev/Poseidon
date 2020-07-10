#ifndef pc104_vitals
#define pc104_vitals


#include "ros/ros.h"
#include "raspberrypi_vitals_msg/sysinfo.h"
#include <iostream>
#include <fstream>
#include <string>
#include "sys/statvfs.h"
using namespace std;

class HBV{
	private:
		
		ros::NodeHandle node;
		ros::Publisher HBVTopic;
		#define _LL_BUFFSIZE_ 2048

		uint32_t sequenceNumber;
		string cputemp;
		string cpustat;
		char ret;
		double loadavg;
		string totalram;
		string freeram;
		string freeram1;
		double f_totalram;
		double f_freeram;
		double f_ram;
		string hdddetail;
		string uptime;
		float upt;

	public:
		HBV(){
			HBVTopic = node.advertise<raspberrypi_vitals_msg::sysinfo>("vitals", 1000);
		}

		void run(){
			ros::Rate loop_rate(1);

		        while(ros::ok()){

                		raspberrypi_vitals_msg::sysinfo msg;

				msg.header=++sequenceNumber;

//******************************CPU temps
				//ifstream fcputemp;
				//fcputemp.open ("/sys/class/thermal/thermal_zone0/temp");
				//if (fcputemp.is_open()){
				//getline (fcputemp,cputemp);
				//fcputemp.close();}
				//cputemp.erase (cputemp.begin()+2);
				//cputemp.insert(2,".");
				//cputemp = température en string xx.xx C
				//msg.cputemp=strtof((cputemp).c_str(),0);
				msg.cputemp=nan("");					
//******************************CPU load
				ifstream fcpustat;
				fcpustat.open ("/proc/loadavg");
				if (fcpustat.is_open()){
				getline (fcpustat,cpustat);
				fcpustat.close();}
				ret = sscanf(cpustat.c_str(), "%lf",&loadavg);
				loadavg = loadavg / 8.0 * 100.0;
				//loadavg = charge cpu sur 100 tenant compte que le raspberry a 4 core et 2 thread par core.
				msg.cpuload=loadavg;
				
//******************************Free RAM
				ifstream ffram;
				ffram.open ("/proc/meminfo");
				if (ffram.is_open()){
				getline (ffram,totalram);
				getline (ffram,freeram1);
				getline (ffram,freeram);
				ffram.close();}
				totalram.erase (totalram.begin(), totalram.begin()+18);
				totalram.erase (totalram.end(), totalram.end()-3);
				freeram.erase (freeram.begin(), freeram.begin()+18);
				freeram.erase (freeram.end(), freeram.end()-3);
				ret = sscanf(totalram.c_str(), "%lf",&f_totalram);
				ret = sscanf(freeram.c_str(), "%lf",&f_freeram);
				f_ram = f_freeram / f_totalram * 100.0;
				msg.freeram=f_ram;

//******************************Free HDD
				struct statvfs stat;

				if (statvfs("/home/", &stat) != 0){
				}
				else {
				
				}

				msg.freehdd=stat.f_bfree * 100.0 / stat.f_blocks ;

//******************************Uptime
				ifstream fuptime;
				fuptime.open ("/proc/uptime");
				if (fuptime.is_open()){
				getline (fuptime,uptime);
				fuptime.close();}
				ret = sscanf(uptime.c_str(), "%f",&upt);
				msg.uptime=upt;
				
//******************************

				msg.vbat=12.2;
				msg.rh=25;
				msg.temp=12;
				msg.psi=64;

				
		                HBVTopic.publish(msg);
                		ros::spinOnce();
                		loop_rate.sleep();
        		}
		}
};


#endif
