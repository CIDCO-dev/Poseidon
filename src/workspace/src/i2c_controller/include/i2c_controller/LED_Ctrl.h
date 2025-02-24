#ifndef LED_Ctrl_H
#define LED_Ctrl_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

// LED states matching original LED_Ctrl enum
enum States {Off, Ready, Recording, Warning, NoFix, Critical};

class LED_Ctrl {
private:
    ros::NodeHandle n;
    ros::Publisher led_pub;
    std::string last_led_state;  // Stores the last known LED state

public:
    LED_Ctrl() {
        // Advertise the LED control topic
        led_pub = n.advertise<std_msgs::String>("led_control", 1);
        last_led_state = "off";  // Default state
    }

    ~LED_Ctrl() {}

    bool set_led(const std::string &led_mode) {
        std_msgs::String msg;
        msg.data = led_mode;
        led_pub.publish(msg);
        last_led_state = led_mode;  // Store the last known state
        //ROS_WARN_STREAM("LED_Ctrl: LED set to " << led_mode);
        return true;
    }

    bool get_state(i2c_controller_service::i2c_controller_service::Response &response) {
        // Convert the last known state to enum
        if (last_led_state == "off") response.value = Off;
        else if (last_led_state == "ready") response.value = Ready;
        else if (last_led_state == "recording") response.value = Recording;
        else if (last_led_state == "warning") response.value = Warning;
        else if (last_led_state == "nofix") response.value = NoFix;
        else if (last_led_state == "error") response.value = Critical;
        else response.value = Off;  // Default to Off if unknown
        return true;
    }
};

#endif
