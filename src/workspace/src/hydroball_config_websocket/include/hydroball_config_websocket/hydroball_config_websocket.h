#ifndef hydroball_config_websocket
#define hydroball_config_websocket

#include <functional>
#include <mutex>
#include <set>
#include <thread>
#include <glob.h>
#include <iostream>
#include <istream>
#include <vector>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <mutex>

#include "ros/ros.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
//std::mutex mtx;
using websocketpp::connection_hdl;

//TODO: move this to a util class

#define D2R(x) (x * ((double)180/(double)M_PI))



class ControlConfig {
public:
    ControlConfig(std::string & logFolder): logFolder(logFolder) {
        srv.init_asio();
        srv.set_reuse_addr(true);
        srv.set_open_handler(bind(&ControlConfig::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&ControlConfig::on_close,this,std::placeholders::_1));
	srv.set_message_handler(bind(&ControlConfig::on_message,this,std::placeholders::_1,std::placeholders::_2));
        logfolder = logFolder;
        }


        void on_message(connection_hdl hdl, server::message_ptr msg) {
	std::string str;
	std::string str1;
	std::string add_lat;
	std::string add_long;
	std::string insert_add;
	data_recived = msg->get_payload();
	str = data_recived;
        str.erase (0,2);
	str.erase (6,(str.length()));

      if (str == "delete") {//supression de fichier log
		str = data_recived;
        	str.erase (0,11);
		str.erase ((str.length()-2),(str.length()));
		str1 = logFolder;
		str1.erase ((str1.length()-1));
		str = str1 + str;
		if( remove(str.c_str()) != 0 ){
    			ROS_INFO( "Error deleting file" );}
  		else{
    			ROS_INFO( "File successfully deleted" );}
		//ROS_INFO(str.c_str());
		
	}
	
	if (str == "config") {//création de la liste de fichier
		 //Build JSON object to send to web interface
            std::stringstream ss;
            ss << "{";

	    //list files and send it over websocket
	    ss << "\"configlist\":[" ;

	    glob_t glob_result;	
	    glob(logFolder.c_str(),GLOB_TILDE,NULL,&glob_result);
	    for(unsigned int i=0; i<glob_result.gl_pathc; ++i){
	    	str = glob_result.gl_pathv[i];
		if (i > 0) {ss << "," ;} 
		ss << "[";
		str.erase (0,(logFolder.length()-1));
		ss << "\"" << str << "\"" ; //file name
		str = glob_result.gl_pathv[i];
	    	str.erase (0,(logFolder.length()-8));
	    	ss << "," ; 	
            	ss << "\"" << str << "\"" ;
		ss << "]";
        	}
     
            ss << "]";  

	

            ss << "}";
            std::lock_guard<std::mutex> lock(mtx);
            for (auto it : connections) {
                 srv.send(it,ss.str(),websocketpp::frame::opcode::text);
            }
		ROS_INFO(str.c_str());
		}
	}


    void on_open(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.insert(hdl);
    }

    void on_close(connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(mtx);
        connections.erase(hdl);
    }

    void receiveMessages(){
        ros::spin();
	
    }

    void run(uint16_t port){
        srv.listen(port);
        srv.start_accept();
        srv.run();
    }
  
    
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server srv;
    con_list connections;
    std::mutex mtx;
    
    ros::NodeHandle n;    
    
    std::string logfolder;
    std::string logFolder;
    uint64_t lastTimestamp;
    std::string data_recived;
    double val_lat;
    double val_long;
  

};



#endif
