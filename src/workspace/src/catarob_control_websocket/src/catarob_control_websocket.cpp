#include "catarob_control_websocket/catarob_control_websocket.h"
    
    
void convertToEulerAngles(const geometry_msgs::Quaternion & q,double & heading,double & pitch, double & roll){
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1){
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else{
        pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    heading = std::atan2(siny_cosp, cosy_cosp);
}
  
