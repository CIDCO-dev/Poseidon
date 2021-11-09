#include "gnss_dummy/gnss_dummy.h"

GNSS::GNSS():sequenceNumber(0){
	gnssTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 1000);
}


double GNSS::ellipsoidalHeight(uint32_t sequenceNumber){
	return sin(sequenceNumber*42+100)*10;
}

void GNSS::message(uint32_t msgSequenceNumber,double longitude,double latitude){
	sensor_msgs::NavSatFix msg;
	msg.header.seq=msgSequenceNumber;
	msg.header.stamp=ros::Time::now();
	msg.status.service = 1;
	msg.status.status = 1;
	msg.header.stamp.nsec=0;
	msg.longitude=longitude;
	msg.latitude=latitude;
	msg.altitude=GNSS::ellipsoidalHeight(msgSequenceNumber);

	gnssTopic.publish(msg);
}

void GNSS::talk(){
	double earthRadius = 6371000;
	double vesselSpeed = 2.0; // m/s, 2 m/s is about 4 knots
	sequenceNumber++;

	longitude += 0.0000001 * (rand() % 10 - 5);
	latitude  += 0.0000001 * (rand() % 10 - 5);
	latitude  = 48.632697 
        + 0.0002 * cos( 6.28 * sequenceNumber / 100 )
        + ( rand()/static_cast<double>(RAND_MAX) - 0.5 ) * 0.00001; 

	longitude = -68.642391 + 0.1 * sin ( 6.28 * sequenceNumber / 100 )
        + ( rand()/static_cast<double>(RAND_MAX) - 0.5 ) * 0.005; 


        // Straight path with a constant longitude
        //longitude = -68;

        double distanceCovered = sequenceNumber / 1 * vesselSpeed;

        latitude  = 48.632697 + R2D(distanceCovered / earthRadius);

	GNSS::message(sequenceNumber,longitude,latitude);
	
}
