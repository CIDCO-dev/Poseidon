#ifndef hydroball_files_websocket
#define hydroball_files_websocket

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

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
//std::mutex mtx;
using websocketpp::connection_hdl;

class ControlFiles {
public:
    ControlFiles(std::string & logFolder): logFolder(logFolder) {
        srv.init_asio();
        srv.set_reuse_addr(true);
        srv.set_open_handler(bind(&ControlFiles::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&ControlFiles::on_close,this,std::placeholders::_1));
	srv.set_message_handler(bind(&ControlFiles::on_message,this,std::placeholders::_1,std::placeholders::_2));
        logfolder = logFolder;
        }

    void deleteFile(std::string & fileToDelete) {
        if( remove(fileToDelete.c_str()) != 0 ){
            ROS_ERROR_STREAM("Error deleting file: " << fileToDelete);
  		} else {
  		    ROS_INFO_STREAM("File successfully deleted: " << fileToDelete);
  		}
    }

    void buildFileListJson(std::string & json) {
        /*
        document.AddMember("telemetry", telemetry, document.GetAllocator());
        rapidjson::StringBuffer sb;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
        document.Accept(writer);
        json = sb.GetString();
        */
        json = "TODO";
        return;
    }

    void buildFileListStringstream(std::stringstream & ss) {
        ss << "{";

	    //list files and send it over websocket
	    ss << "\"fileslist\":[" ;

	    glob_t glob_result;
	    glob(logFolder.c_str(),GLOB_TILDE,NULL,&glob_result);
	    for(unsigned int i=0; i<glob_result.gl_pathc; ++i){
	    	std::string str = glob_result.gl_pathv[i];
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
    }

    void sendFileList() {
        //Build JSON object to send to web interface
        std::stringstream ss;

        buildFileListStringstream(ss);
        std::string toSend = ss.str();

        std::lock_guard<std::mutex> lock(mtx);
        for (auto it : connections) {
             srv.send(it,toSend,websocketpp::frame::opcode::text);
        }

		ROS_INFO(toSend.c_str());
    }

    void on_message(connection_hdl hdl, server::message_ptr msg) {
		rapidjson::Document document;

		if(document.Parse(msg->get_payload().c_str()).HasParseError()){
			//Not valid JSON
            rapidjson::StringBuffer sb;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
            document.Accept(writer);
            std::string jsonString = sb.GetString();
			ROS_ERROR_STREAM("Error parsing JSON on_message: " << jsonString);
			return;
		}

		if(document.HasMember("delete") && document["delete"].IsArray()) {
		    std::string fileToDelete = document["delete"][0].GetString();
		    deleteFile(fileToDelete);
		} else if(document.HasMember("f-list")) {
		    sendFileList();
		} else {
		    //no command found. ignore
			ROS_ERROR("No command found");
		}
	}


        void on_messageOld(connection_hdl hdl, server::message_ptr msg) {
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
	
	if (str == "f-list") {//creation de la liste de fichier
		 //Build JSON object to send to web interface
            std::stringstream ss;
            ss << "{";

	    //list files and send it over websocket
	    ss << "\"fileslist\":[" ;

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

/*
    void receiveMessages(){
        ros::spin();

    }
    */

    void run(uint16_t port){
        srv.listen(port);
        srv.start_accept();
        srv.run();
    }

    void stop() {
	    websocketpp::lib::error_code ec_stop_listening;
	    srv.stop_listening(ec_stop_listening);
	    if(ec_stop_listening) {
	        ROS_ERROR_STREAM("failed to stop listening: " << ec_stop_listening.message());
	        return;
	    }

	    std::string closingMessage = "Server has closed the connection";
	    std::lock_guard<std::mutex> lock(mtx); // server stopped listening is this needed?
	    for (auto it : connections) {
            	websocketpp::lib::error_code ec_close_connection;
            	srv.close(it,websocketpp::close::status::normal,closingMessage, ec_close_connection);
            	if(ec_close_connection) {
                	ROS_ERROR_STREAM("failed to close connection: " << ec_close_connection.message());
             	}
            }

	    ROS_INFO("Stopping Files server");
            srv.stop();
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
  

};



#endif
