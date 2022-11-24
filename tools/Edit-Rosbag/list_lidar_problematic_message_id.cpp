#include <iostream>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"



int main(int argc, char** argv) {

if(argc != 2){
	std::cerr<<"./llpmi bagname \n";
}

std::string bag_filename = argv[1];

std::cout<<bag_filename << "\n";

rosbag::Bag bag;
bag.open(bag_filename, rosbag::bagmode::Read);
rosbag::View view(bag);

size_t message_index = 0;

for (const rosbag::MessageInstance& message : view) {
    ++message_index;
    std::string frame_id;

	if (message.isType<sensor_msgs::PointCloud2>()) {
		//std::cout<<lidarMsgs<<"\n";
		//lidarMsgs++;
		auto msg = message.instantiate<sensor_msgs::PointCloud2>();
		//time = msg->header.stamp;
		//frame_id = msg->header.frame_id;
		range_data_checker.CheckMessage(*msg);
	}
}


return 0;
}
