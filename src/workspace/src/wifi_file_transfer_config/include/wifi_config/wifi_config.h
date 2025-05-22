#ifndef wifi_config
#define wifi_config

// ROS
#include "ros/ros.h"

// Poseidon
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"
#include "../../utils/string_utils.hpp"

// C++ std
#include <iostream>
#include <filesystem>
#include <sstream>
#include <fstream>
#include <map>

class WifiConfig {
private:
    ros::NodeHandle node;
    ros::Subscriber configurationSubscriber;
    ros::ServiceClient configurationClient;
    std::map<std::string, std::map<std::string, std::string>> wifiConnections;

    std::string currentSsid;
    std::string currentPassword;
    bool autoconnectStatus;

    unsigned int configCallbackCounter = 0;

    void print_wifi_connections() {
        for (const auto& connection : wifiConnections) {
            const auto& config = connection.second;
            ROS_INFO_STREAM("Stored WiFi: " << connection.first << ", autoconnect=" << config.at("autoconnect") << ", password=" << config.at("password"));
        }
    }

    void get_auto_connect_status() {
        char buffer[256];
        std::string line, ssid, autoconnect;
        bool firstLine = true;

        ROS_INFO("Fetching autoconnect status from nmcli...");
        std::shared_ptr<FILE> pipe(popen("nmcli -f NAME,AUTOCONNECT con show", "r"), pclose);
        if (!pipe) {
            ROS_ERROR("Failed to run nmcli for autoconnect status");
            return;
        }
        while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
            line = buffer;
            if (firstLine) {
                firstLine = false;
                continue;
            }
            std::istringstream iss(line);
            if (iss >> ssid >> autoconnect) {
                auto& config = wifiConnections[ssid];
                config["autoconnect"] = autoconnect;
                ROS_INFO_STREAM("Autoconnect: " << ssid << " = " << autoconnect);
            } else {
                ROS_WARN_STREAM("Failed to parse line: " << line);
            }
        }
    }

    void add_connection_infos(const std::string &path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            ROS_WARN_STREAM("Could not open connection file: " << path);
            return;
        }

        std::string line, ssid, password, id;
        while (std::getline(file, line)) {
            line = trimSpaces(line);
            if (line.rfind("ssid=", 0) == 0) ssid = trimSpaces(line.substr(5));
            else if (line.rfind("psk=", 0) == 0) password = trimSpaces(line.substr(4));
            else if (line.rfind("id=", 0) == 0) id = trimSpaces(line.substr(3));
        }
        file.close();

        if (ssid == id) {
            wifiConnections[ssid]["password"] = password;
            ROS_INFO_STREAM("Loaded connection: " << ssid);
        } else {
            ROS_WARN_STREAM("SSID and ID mismatch in file: " << path);
        }
    }

    void get_nmcli_connections() {
        ROS_INFO("Scanning NetworkManager system-connections directory...");
        try {
            for (const auto& entry : std::filesystem::directory_iterator("/etc/NetworkManager/system-connections/")) {
                add_connection_infos(entry.path());
            }
        } catch (const std::filesystem::filesystem_error& err) {
            ROS_ERROR_STREAM("Filesystem error while accessing connections: " << err.what());
        }
        get_auto_connect_status();
    }

    void add_connection_2_nmcli(const std::string &ssid, const std::string &password, const std::string &autoconnect) {
        std::string command = "nmcli connection add type wifi ifname wlan0 ssid \"" + ssid + "\" ";
        command += "con-name \"" + ssid + "\" wifi-sec.key-mgmt wpa-psk wifi-sec.psk \"" + password + "\" ";
        command += "autoconnect " + autoconnect;

        ROS_INFO_STREAM("Adding WiFi connection: " << command);
        if (std::system(command.c_str()) != 0) {
            ROS_ERROR_STREAM("Failed to add WiFi connection for SSID: " << ssid);
        }

        std::string dns_cmd = "nmcli connection modify \"" + ssid + "\" ipv4.dns \"8.8.8.8\"";
        ROS_INFO_STREAM("Setting DNS for connection: " << dns_cmd);
        std::system(dns_cmd.c_str());
    }

    void modify_nmcli_connection(const std::string &ssid, const std::string &password, const std::string &autoconnect) {
        std::string cmd = "nmcli con modify \"" + ssid + "\" wifi-sec.psk \"" + password + "\" autoconnect " + autoconnect;
        ROS_INFO_STREAM("Modifying WiFi connection: " << cmd);
        std::system(cmd.c_str());

        std::string dns_cmd = "nmcli connection modify \"" + ssid + "\" ipv4.dns \"8.8.8.8\"";
        ROS_INFO_STREAM("Setting DNS for connection: " << dns_cmd);
        std::system(dns_cmd.c_str());

        cmd = "nmcli con down \"" + ssid + "\"";
        ROS_INFO_STREAM("Disconnecting: " << cmd);
        if (std::system(cmd.c_str()) == 0) {
            cmd = "nmcli con up \"" + ssid + "\"";
            ROS_INFO_STREAM("Reconnecting: " << cmd);
            std::system(cmd.c_str());
        } else {
            ROS_WARN_STREAM("Could not bring down connection: " << ssid);
        }
    }

    void activate_connection_on_interface(const std::string &ssid, const std::string &interface) {
        std::string modify_cmd = "nmcli connection modify \"" + ssid + "\" ifname " + interface;
        ROS_INFO_STREAM("Assigning connection to interface: " << modify_cmd);
        if (std::system(modify_cmd.c_str()) != 0) {
            ROS_ERROR_STREAM("Failed to assign connection to interface " << interface);
            return;
        }

        std::string up_cmd = "nmcli connection up \"" + ssid + "\"";
        ROS_INFO_STREAM("Bringing up connection: " << up_cmd);
        if (std::system(up_cmd.c_str()) != 0) {
            ROS_ERROR_STREAM("Failed to bring up connection: " << ssid);
        }
    }

    void get_wifi_config() {
        setting_msg::ConfigurationService srv;

        srv.request.key = "wifiSSID";
        if (configurationClient.call(srv)) currentSsid = srv.response.value;

        srv.request.key = "wifiPassword";
        if (configurationClient.call(srv)) currentPassword = srv.response.value;

        srv.request.key = "wifiTransferEnabled";
        if (configurationClient.call(srv)) autoconnectStatus = (srv.response.value == "true" || srv.response.value == "yes");

        ROS_INFO_STREAM("Config: SSID=" << currentSsid << ", AutoConnect=" << (autoconnectStatus ? "yes" : "no"));

        if (wifiConnections.find(currentSsid) == wifiConnections.end()) {
            std::map<std::string, std::string> config;
            config["autoconnect"] = autoconnectStatus ? "yes" : "no";
            config["password"] = currentPassword;
            wifiConnections[currentSsid] = config;
            add_connection_2_nmcli(currentSsid, currentPassword, config["autoconnect"]);
        } else {
            modify_nmcli_connection(currentSsid, currentPassword, autoconnectStatus ? "yes" : "no");
        }

        activate_connection_on_interface(currentSsid, "wlan0");
    }

    void configurationCallBack(const setting_msg::Setting &setting) {
        if (setting.key == "wifiSSID") {
            configCallbackCounter++;
            if (!setting.value.empty()) currentSsid = setting.value;
        } else if (setting.key == "wifiPassword") {
            configCallbackCounter++;
            if (setting.value.length() >= 8) currentPassword = setting.value;
        } else if (setting.key == "wifiTransferEnabled") {
            configCallbackCounter++;
            autoconnectStatus = (setting.value == "true" || setting.value == "yes");
        }

        if (configCallbackCounter >= 3) {
            ROS_INFO_STREAM("Applying full WiFi configuration for SSID: " << currentSsid);
            if (wifiConnections.find(currentSsid) == wifiConnections.end()) {
                wifiConnections[currentSsid] = {
                    {"autoconnect", autoconnectStatus ? "yes" : "no"},
                    {"password", currentPassword}
                };
                add_connection_2_nmcli(currentSsid, currentPassword, autoconnectStatus ? "yes" : "no");
            } else {
                modify_nmcli_connection(currentSsid, currentPassword, autoconnectStatus ? "yes" : "no");
            }
            activate_connection_on_interface(currentSsid, "wlan0");
            configCallbackCounter = 0;
        }
    }

public:
    WifiConfig() {
        configurationSubscriber = node.subscribe("configuration", 1000, &WifiConfig::configurationCallBack, this);
        configurationClient = node.serviceClient<setting_msg::ConfigurationService>("get_configuration");
        configurationClient.waitForExistence();
        get_nmcli_connections();
        get_wifi_config();
    }

    ~WifiConfig() {}
};

#endif