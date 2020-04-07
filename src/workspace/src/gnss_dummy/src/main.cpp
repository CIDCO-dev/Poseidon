#include "gnss_dummy/main.h"

#define PI M_PI
#define R2D ((double)180/(double)PI)

class GNSS{
	private:
		ros::NodeHandle node;
		ros::Publisher gnssTopic;

		uint32_t sequenceNumber;

		double longitude;
		double latitude;
		double ellipsoidalHeight;

	public:
		GNSS(){
			gnssTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 1000);
		}

		void run(){

            double rosRate = 1; //10.0;

			ros::Rate loop_rate( rosRate );

            double earthRadius = 6371000;

            double vesselSpeed = 2.0; // m/s, 2 m/s is about 4 knots

		    while(ros::ok()){

                sensor_msgs::NavSatFix msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

                msg.status.service = 1;

/*
				msg.header.stamp.nsec=0;

				longitude += 0.0000001 * (rand() % 10 - 5);
				latitude  += 0.0000001 * (rand() % 10 - 5);
*/

/*
                latitude  = 48.632697 
                            + 0.0002 * cos( 6.28 * sequenceNumber / 100 )
                            + ( rand()/static_cast<double>(RAND_MAX) - 0.5 ) * 0.00001; 

				longitude = -68.642391 + 0.1 * sin ( 6.28 * sequenceNumber / 100 )
                            + ( rand()/static_cast<double>(RAND_MAX) - 0.5 ) * 0.005; 
*/

                // Straight path with a constant longitude
                longitude = -68;

                double distanceCovered = sequenceNumber / rosRate * vesselSpeed;

                latitude  = 48.632697 + distanceCovered / earthRadius * R2D;

                std::cout << std::setprecision(10) << std::fixed
                    << "gnss_dummy(), sequenceNumber: " << sequenceNumber
                    << ", distanceCovered: " << distanceCovered 
                    << ", latitude: " << latitude << std::endl;


				double ellipsoidalHeight    = sin(sequenceNumber*42+100)*10;

				msg.longitude=longitude;
				msg.latitude=latitude;
				msg.altitude=ellipsoidalHeight;

		                gnssTopic.publish(msg);
                		ros::spinOnce();
                		loop_rate.sleep();
        		}
		}
}