#ifndef hydroball_config_websocket
#define hydroball_config_websocket

#include <functional>
#include <mutex>
#include <set>
#include <thread>
#include <map>
#include <iostream>
#include <fstream>
#include <istream>
#include <vector>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <mutex>
#include <string>

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

  

	if (str == "newcfg") {//modification de la configuration
	    //ROS_WARN("New Config Reviced");
	    //ROS_WARN(data_recived.c_str());	
	    //structure des donnée recu pour générer une string pour fichier txt
            str = data_recived;
	    str.erase (0,10);
	    std::ofstream mycfg ;
	    mycfg.open(logFolder.c_str()); 
	    while (str.find('[')){
		val1start = str.find('"');
		str.replace(val1start, 1, "+");
		val1stop  = str.find('"');
		str.replace(val1stop, 1,  "+");
		val2start = str.find('"');
		str.replace(val2start, 1,  "+");
		val2stop  = str.find('"');
		str.replace(val2stop, 1,  "+");
		stra = str;
		strb = str;
		stra.erase (val1stop,(stra.length()));
		strb.erase (val2stop,(strb.length()));
		stra.erase (0,val1start+1);
		strb.erase (0,val2start+1);
		str.erase (0,val2stop);
		//ROS_WARN(str.c_str());
		//ROS_WARN(stra.c_str());
		//ROS_WARN(strb.c_str());
		strout = stra + "=" + strb;
		//ROS_WARN(strout.c_str());
		mycfg << strout;
		mycfg << "\n";
		if (str.length() < 10) {break;} 
		}
	    mycfg.close();


	    //écriture du fichier txt
	}

	
	if (str == "config") {//création de la liste de configuration
	    //ouverture du fichier config.txt
	    std::ifstream mycfg ;
	    mycfg.open(logFolder.c_str()); 
	    std::string line;

	    //décodage du fichier config.txt
	    if (mycfg.is_open()){
	
		mapline=0;
		while (std::getline(mycfg, line)){
		    mapline++;
		    stra = line;
		    strb = line;
		    stra.erase ((line.find('=')),(line.length()));
		    strb.erase (0 ,(line.find('=')+1));
		    name_map[mapline] = stra;
		    config_map[mapline] = strb;
    		    }
	    	mycfg.close();
	    }
    
	    //préparation de la string de transmission
            std::stringstream ss;
            ss << "{";

	    //list config and send it over websocket
	    ss << "\"configlist\":[" ;
	    for(int i=1; i<mapline+1; ++i){
	    	str =  name_map[i];
		if (i > 1) {ss << "," ;} 
		ss << "[";
		ss << "\"" << str << "\"" ; //file name
		str = config_map[i];
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
		//ROS_INFO(str.c_str());
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
    std::map<int, std::string> name_map;
    std::map<int, std::string> config_map;
    std::map<int, std::string> new_name_map;
    std::map<int, std::string> new_config_map;

    int mapline;
    std::string stra;
    std::string strb;
    std::string strout;
    std::size_t val1start;
    std::size_t val1stop;
    std::size_t val2start;
    std::size_t val2stop;
};



#endif
