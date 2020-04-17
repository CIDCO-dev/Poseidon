#ifndef hydroball_control_websocket
#define hydroball_control_websocket

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

#include "state_controller/State.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
std::mutex mtx;
using websocketpp::connection_hdl;

//TODO: move this to a util class

#define D2R(x) (x * ((double)180/(double)M_PI))


void convertToEulerAngles(const geometry_msgs::Quaternion & q,double & heading,double & pitch, double & roll);

class ControlServer {
public:
    ControlServer(std::string & logFolder): logFolder(logFolder) {
        srv.init_asio();
        srv.set_reuse_addr(true);
        srv.set_open_handler(bind(&ControlServer::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&ControlServer::on_close,this,std::placeholders::_1));
	srv.set_message_handler(bind(&ControlServer::on_message,this,std::placeholders::_1,std::placeholders::_2));
        logfolder = logFolder;
        stateTopic = n.subscribe("state", 1000, &ControlServer::stateChanged,this);
    }


        void number_err(){
	//Build JSON object to send to web interface
        std::stringstream ss;
	ss << "{\"number_err\":[1]}" ;
        std::lock_guard<std::mutex> lock(mtx);
        for (auto it : connections) {
        	srv.send(it,ss.str(),websocketpp::frame::opcode::text);
		}
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
       	if (str == "go_del") {//supression de goal pour le goalplaner
		mtx.lock();
		str = data_recived;
        	str.erase (0,11);
		str.erase ((str.length()-2),(str.length()));
		int x = std::stoi(str);
		goal_planner_lat.erase(goal_planner_lat.begin()+x);
    		goal_planner_long.erase(goal_planner_long.begin()+x);
    		mtx.unlock();

		//ROS_WARN(str.c_str());
		}
	if (str == "go_add") {//ajout de goal pour le goalplaner
		mtx.lock();
		str = data_recived;
        	str.erase (0,11);
		str.erase ((str.length()-2),(str.length()));
		add_lat = str;
		add_long = str;
		std::size_t found = str.find(',');
		add_lat.erase (found, str.length());
		add_long.erase(0, (found + 1));
		val_lat = std::stod(add_lat);
		val_long = std::stod(add_long);
		//ROS_WARN(str.c_str());
		//ROS_WARN(add_lat.c_str());
		//ROS_WARN(add_long.c_str());
		if( add_lat.find_first_not_of("1234567890.-") == std::string::npos )
    		{
		goal_planner_lat.push_back(val_lat);
		}else{
		number_err();
		}
		if( add_long.find_first_not_of("1234567890.-") == std::string::npos )
    		{
		goal_planner_long.push_back(val_long);
		}else{
		number_err();
		}


    		mtx.unlock();
		}
	if (str == "go_edi") {
		mtx.lock();
		str = data_recived;
        	//str.erase (0,11);
		//str.erase ((str.length()-2),(str.length()));
		//int x = std::stoi(str);
		//goal_planner_lat.erase(goal_planner_lat.begin()+x);
    		//goal_planner_long.erase(goal_planner_long.begin()+x);
    		mtx.unlock();

		ROS_WARN(str.c_str());
		}
	if (str == "go_ins") {//insertion d'une valeur dans le goalplanner
		mtx.lock();
		str = data_recived;
        	str.erase (0,11);
		str.erase ((str.length()-2),(str.length()));
		insert_add = str;
		std::size_t found1 = str.find(',');
		insert_add.erase (found1, str.length());
		str.erase(0, (found1 + 1));
		add_lat = str;
		add_long = str;
		std::size_t found2 = str.find(',');
		add_lat.erase (found2, str.length());
		add_long.erase(0, (found2 + 1));
		int x = std::stoi(insert_add);
		val_lat = std::stod(add_lat);
		val_long = std::stod(add_long);
		//ROS_WARN(str.c_str());
		//ROS_WARN(insert_add.c_str());
		//ROS_WARN(add_lat.c_str());
		//ROS_WARN(add_long.c_str());
		if( add_lat.find_first_not_of("1234567890.-") == std::string::npos )
    		{
		goal_planner_lat.insert (goal_planner_lat.begin()+x,std::stod(add_lat));
		}else{
		number_err();
		}
		if( add_long.find_first_not_of("1234567890.-") == std::string::npos )
    		{
		goal_planner_long.insert(goal_planner_long.begin()+x,std::stod(add_long));
		}else{
		number_err();
		}
		    		
    		mtx.unlock();
		
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



        void stateChanged(const state_controller::State & state) {
        
        uint64_t timestamp = (state.attitude.header.stamp.sec * 1000000) + (state.attitude.header.stamp.nsec/1000);
        std::string str;
        if(
                //TODO: maybe add our own header?
                (timestamp - lastTimestamp > 200000)
                //&&
                //()
        ){
            //Build JSON object to send to web interface
            std::stringstream ss;
            ss << "{";

            
       
      

	    //goalplanner value
	    ss << "\"goal_planner\":[" ;
	    for(unsigned int i=0; i<goal_planner_lat.size(); ++i){
		if (i > 0) {ss << "," ;} 
		ss << "[";
		std::string strObj5 = "0.0";
		try
		{
			strObj5 = boost::lexical_cast<std::string>(goal_planner_lat.at(i));
		}
		catch (boost::bad_lexical_cast const& e)
		{
			std::cout << "Error: " << e.what() << "\n";
		}
		ss << strObj5;
		ss << ",";
		ss << std::to_string(goal_planner_long.at(i));
                ss << "]";
		}
	    ss << "]";     


            ss << "}";
            std::lock_guard<std::mutex> lock(mtx);
            for (auto it : connections) {
                 srv.send(it,ss.str(),websocketpp::frame::opcode::text);
            }

	  

            lastTimestamp = timestamp;
        }
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
    ros::Subscriber stateTopic;
    std::string logfolder;
    std::string logFolder;
    uint64_t lastTimestamp;
    std::string data_recived;
    double val_lat;
    double val_long;	
    std::vector<double> goal_planner_lat = {-68.504926667,-68.504926666,-68.504926665,-68.504926664};
    std::vector<double> goal_planner_long = {48.437141667,48.437141667,48.437141667,48.437141667};
    

};



#endif
