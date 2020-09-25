#include "imu_dummy/imu_dummy.h"
#include "../../utils/QuaternionUtils.h"

void IMU::run(){
    nav_msgs::Odometry msg;

    msg.header.seq=++sequenceNumber;
    msg.header.stamp=ros::Time::now();

    double heading = sin(sequenceNumber/360)*30;
    double pitch   = cos(sequenceNumber/360)*20;
    double roll    = sin(sequenceNumber/360)*10;

    QuaternionUtils::convertToQuaternion(heading,pitch,roll,msg);

    imuTopic.publish(msg);
}

void IMU::message(uint32_t sequenceNumber,double yaw, double pitch, double roll){
    nav_msgs::Odometry msg;
    msg.header.seq=sequenceNumber;
    msg.header.stamp=ros::Time::now();
    QuaternionUtils::convertToQuaternion(yaw,pitch,roll,msg);
    imuTopic.publish(msg);
}
