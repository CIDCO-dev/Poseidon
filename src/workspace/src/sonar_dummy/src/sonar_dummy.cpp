#include "sonar_dummy/sonar_dummy.h"


		void Sonar::run(){
				geometry_msgs::PointStamped msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

				msg.point.z = sin(sequenceNumber)*30;

		                sonarTopic.publish(msg);
                	
        	
		}
