#include "sonar_dummy/sonar_dummy.h"


void Sonar::message(double z) {
    geometry_msgs::PointStamped msg;
    msg.header.seq=++sequenceNumber;
    msg.header.stamp=ros::Time::now();
    msg.point.z = z;
    sonarTopic.publish(msg);
}

void Sonar::run(){
    message(sin(sequenceNumber)*30);
}
