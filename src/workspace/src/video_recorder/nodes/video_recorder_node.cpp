#include "ros/ros.h"
#include "../../utils/timestamp.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sensor_msgs/NavSatFix.h"


class VideoProcessor{

    public:
    	VideoProcessor(std::string outputFolder){
			this->outputFolder = outputFolder;
			this->cap.open(0);

    	};
    	
    	~VideoProcessor(){};
    	
    	void run(){
    		
			if(!cap.isOpened()){
				ROS_ERROR_STREAM("Error opening video stream or file");
			}

			while(ros::ok()){

				cv::Mat frame;
				// Capture frame-by-frame
				cap.read(frame);
				//if (frame.empty()) break;
				
				//cv::imshow( "Frame", frame );
				// Press  ESC on keyboard to exit
				//char c=(char)cv::waitKey(1);
				//if(c==27) break;
				
				std::string filename = this->tempFolder + "/image";
				filename += std::to_string(count);
				cv::imwrite(filename+".png", frame);
				this->count++;
				ros::spinOnce();
    		}
		};
		    	
    	void interpolateCoord();
    	
    	void gnssCallback(const sensor_msgs::NavSatFix& gnss){
    		
    		ROS_ERROR_STREAM("video_recorder_node gnss callBack");
    		
    		
    	};
    	//void imuCallback();
    	
    	

	private:
		//ros::NodeHandle node;
		std::string outputFolder;
		const std::string tempFolder = "/home/ubuntu/temp";
		unsigned int count = 0;
		cv::VideoCapture cap;
	
};


int main(int argc, char **argv)
{
	if(argc < 2){

		std::cout << "video recorder, Missing output folder path" << std::endl;
		return 1;
	}

	try{
		ros::init(argc, argv, "video_recorder");

		ros::NodeHandle n;

		std::string outputFolder( argv[1] );

		VideoProcessor videoProcessor(outputFolder);
		

		ros::Subscriber sub1 = n.subscribe("fix", 1000, &VideoProcessor::gnssCallback,&videoProcessor);
		//ros::Subscriber sub2 = n.subscribe("imu/data", 1000, &VideoProcessor::imuCallback,&videoProcessor);
		videoProcessor.run();
		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
