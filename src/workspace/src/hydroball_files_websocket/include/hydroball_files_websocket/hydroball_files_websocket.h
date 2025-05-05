#ifndef hydroball_files_websocket
#define hydroball_files_websocket

#include <functional>
#include <mutex>
#include <set>
#include <thread>
//#include <glob.h>
#include <iostream>
#include <istream>
#include <vector>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <curl/curl.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "logger_service/TriggerTransfer.h"

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;

class ControlFiles {
public:
    ControlFiles(std::string & logFolder): logFolder(logFolder) {
    	ROS_INFO_STREAM("log folder: "<<logFolder);
        srv.init_asio();
        srv.set_reuse_addr(true);
		srv.clear_access_channels(websocketpp::log::alevel::all);
        srv.set_open_handler(bind(&ControlFiles::on_open,this,std::placeholders::_1));
        srv.set_close_handler(bind(&ControlFiles::on_close,this,std::placeholders::_1));
		srv.set_message_handler(bind(&ControlFiles::on_message,this,std::placeholders::_1,std::placeholders::_2));
    }

    void deleteFile(std::string & fileToDelete) {
        if( remove(fileToDelete.c_str()) != 0 ){
            ROS_ERROR_STREAM("Error deleting file: " << fileToDelete);
  		} else {
  		    ROS_INFO_STREAM("File successfully deleted: " << fileToDelete);
  		}
    }

std::vector<std::string> getFilesToTransferFromDisk() {
    std::vector<std::string> fichiers;
    std::filesystem::path PATH = logFolder;
    for (const auto& dir_entry : std::filesystem::directory_iterator{PATH}) {
        if (dir_entry.is_regular_file()) {
            fichiers.push_back(dir_entry.path().string());
        }
    }
    return fichiers;
}

    void buildFileListJson(rapidjson::Document & document) {
    	//rapidjson::Document document;
		document.SetObject();
		rapidjson::Value filePaths(rapidjson::kArrayType);
		rapidjson::Document::AllocatorType& allocator = document.GetAllocator();
		
    	//std::string path = "/home/ubuntu/Poseidon/www/webroot/record";
		std::filesystem::path PATH = logFolder;
		for(auto &dir_entry: std::filesystem::directory_iterator{PATH}){
		    if(dir_entry.is_regular_file()){
				rapidjson::Value PATH(rapidjson::kArrayType);
				rapidjson::Value PATH2(rapidjson::kStringType);
				char buffer[255];
				int len = sprintf(buffer, "%s",dir_entry.path().filename().string().c_str()); // dynamically created string.
				PATH2.SetString(buffer, len, document.GetAllocator());
				PATH.PushBack(PATH2, allocator);
				
				std::string record = "record/" + dir_entry.path().filename().string();
				len = sprintf(buffer, "%s",record.c_str());
				PATH2.SetString(buffer, len, document.GetAllocator());
				PATH.PushBack(PATH2, allocator);
				
				filePaths.PushBack(PATH, allocator);
			}
		}
		document.AddMember("fileslist",filePaths,document.GetAllocator());
        
        return;
    }

    void sendFileList() {
    	
    	rapidjson::Document document;
		document.SetObject();
		
		buildFileListJson(document);
		
		rapidjson::StringBuffer sb;
		rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
		document.Accept(writer);
        std::string json = sb.GetString();
        std::lock_guard<std::mutex> lock(mtx);
        for (auto it : connections) {
             srv.send(it,json.c_str(),websocketpp::frame::opcode::text);             
        }

}


std::string getApiServerAddress() {
    std::ifstream file("/opt/Poseidon/config.txt");
    std::string line;

    if (!file.is_open()) {
        ROS_ERROR_STREAM("Fail to open /opt/Poseidon/config.txt");
        return "";
    }

    while (std::getline(file, line)) {
        ROS_INFO_STREAM("Line read : " << line);
        if (line.rfind("apiServer", 0) == 0) {
            std::string value = line.substr(std::string("apiServer").length());

            // Space trim
            value.erase(0, value.find_first_not_of(" \t\r\n"));
            value.erase(value.find_last_not_of(" \t\r\n") + 1);

            ROS_INFO_STREAM("Value : '" << value << "'");

            // Vérifie si vide, localhost, ou seulement des espaces
            if (value.empty() || value == "localhost") {
                ROS_ERROR_STREAM("apiServer empty or localhost");
                return "";
            }

            ROS_INFO_STREAM("Api Server address : " << value);
            return value;
        }
    }

    ROS_ERROR_STREAM("key 'apiServer=' not found in /opt/Poseidon/config.txt");
    return "";
}



bool pingAddress(const std::string& address) {
    std::string command = "ping -c 1 -W 1 " + address + " > /dev/null 2>&1";
    return (system(command.c_str()) == 0);
}


bool isInternetAvailable() {
    return pingAddress("google.com");
}

bool isApiAvailable() {
    std::string domain = getApiServerAddress();
    if (domain.empty() || domain == "localhost") return false;

    std::string url = "https://" + domain + "/";  
    CURL *curl = curl_easy_init();
    if (!curl) return false;

    CURLcode res;
    long http_code = 0;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);      // Timeout rapide
    curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);        // HEAD only
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L); // Pour ignorer les certificats invalides si nécessaire
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L); // Idem

    res = curl_easy_perform(curl);
    if (res == CURLE_OK) {
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    }

    curl_easy_cleanup(curl);

return (http_code == 200 || http_code == 304);
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

		if(document.HasMember("delete")) {
		    std::string fileToDelete = logFolder + document["delete"].GetString();
		    deleteFile(fileToDelete);
		} else if(document.HasMember("f-list")) {
		    sendFileList();
		} 
		else if (document.HasMember("publishfiles")) {
    std::thread([this, hdl]() {
        try {
            sendStatus(hdl, "Checking files to transfer...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

std::vector<std::string> fichiers = getFilesToTransferFromDisk();
            if (fichiers.empty()) {
                sendStatus(hdl, "❌ No files to transfer", true);
                return;
            }

            sendStatus(hdl, "Checking internet connection...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (!isInternetAvailable()) {
                sendStatus(hdl, "❌ No internet connection", true);
                return;
            }

            sendStatus(hdl, "Checking API server connection...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (!isApiAvailable()) {
                sendStatus(hdl, "❌ No API server connection", true);
                return;
            }

            sendStatus(hdl, "Starting files transfer...");
ros::ServiceClient client = n.serviceClient<logger_service::TriggerTransfer>("trigger_transfer");
logger_service::TriggerTransfer srv;
if (client.call(srv)) {
    sendStatus(hdl, "✅ Transfer done : " + srv.response.message, srv.response.success);
} else {
    sendStatus(hdl, "❌ Fail to start the file transfer", true);
}


        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Error in file pub. : " << e.what());
            sendStatus(hdl, "❌ Error during file transfer", true);
        }
    }).detach();
}

		else {
		    //no command found. ignore
			ROS_ERROR("No command found");
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


void sendStatus(connection_hdl hdl, const std::string& message, bool done = false, const std::string& extraFieldKey = "", const std::string& extraFieldValue = "") {
    rapidjson::Document d;
    d.SetObject();
    rapidjson::Document::AllocatorType& a = d.GetAllocator();

    d.AddMember("publishstatus", rapidjson::Value(message.c_str(), a), a);
    if (done) {
        d.AddMember("done", true, a);
    }

    if (!extraFieldKey.empty()) {
        d.AddMember(rapidjson::Value(extraFieldKey.c_str(), a), rapidjson::Value(extraFieldValue.c_str(), a), a);
    }

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    d.Accept(writer);

    std::string payload = buffer.GetString();

    std::lock_guard<std::mutex> lock(mtx);
    srv.send(hdl, payload, websocketpp::frame::opcode::text);
}


    typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

    server srv;
    con_list connections;
    std::mutex mtx;
    
    ros::NodeHandle n;    
    ros::Subscriber stateTopic;
    //std::string glob_logFolder;
    std::string logFolder;
    uint64_t lastTimestamp;
    std::string data_recived;
    double val_lat;
    double val_long;
  

};



#endif
