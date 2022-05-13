#include "ros/ros.h"
#include "../../utils/timestamp.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sensor_msgs/NavSatFix.h"
#include <filesystem>
#include <algorithm>
#include <exiv2/exiv2.hpp>


class VideoProcessor{

    public:
    	VideoProcessor(std::string outputFolder){
    		if( *(outputFolder.end()) != '/'){
    			outputFolder += "/";
    		}
			this->outputFolder = outputFolder;
			this->cap.open(0);

    	};
    	
    	~VideoProcessor(){};
    	
    	void run(){
    		
			if(!this->cap.isOpened()){
				ROS_ERROR_STREAM("Error opening video stream or file");
			}

			while(ros::ok()){

				cv::Mat frame;
				this->cap.read(frame);
				
				if(coord2Interpolate.size() >= 1){
					std::string filename = this->tempFolder + "image";
					filename += std::to_string(count);
					cv::imwrite(filename+".png", frame);
					this->count++;
				}

				ros::spinOnce();
    		}
		};
		    	
    	void interpolateCoord(){
    	
    		std::vector<std::string> files;
    		std::vector<double> xyz0 = coord2Interpolate.front();
    		std::vector<double> xyz1 = coord2Interpolate.back();
    		std::filesystem::path PATH = tempFolder;
    		auto dirIter = std::filesystem::directory_iterator{PATH};
			int fileCount = 0;
			
			//put files in vector for sorting
			for(auto& entry : dirIter){
				fileCount++;
				files.push_back(entry.path().filename().string());
				
			}

			//considering that within one seconds the speed wont change	
			double x0 = xyz0[0];
			double y0 = xyz0[1];
			double z0 = xyz0[2];
    		double jumpX = (x0 - xyz1[0]) / fileCount;
    		double jumpY = (y0 - xyz1[1]) / fileCount;
    		double jumpZ = (z0 - xyz1[2]) / fileCount;
			
			std::sort(files.begin(), files.end());
			
			//add exif data and move file
    		for(auto const& file : files){
				x0 += jumpX;
				y0 += jumpY;
				z0 += jumpZ;

				Exiv2::ExifData exifData;
 				exifData["Exif.GPSInfo.GPSMapDatum"] = "WGS-84";
 				
 				Exiv2::Value::AutoPtr v = Exiv2::Value::create(Exiv2::asciiString);
 				v->read(std::to_string(x0));
 				Exiv2::ExifKey lon("Exif.GPSInfo.GPSLongitude");
 				exifData.add(lon, v.get());
 				
 				v->read(std::to_string(y0));
 				Exiv2::ExifKey lat ("Exif.GPSInfo.GPSLatitude");
 				exifData.add(lat, v.get());
 				
 				v->read(std::to_string(z0));
 				Exiv2::ExifKey alt("Exif.GPSInfo.GPSAltitude");
 				exifData.add(alt, v.get());

				Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(tempFolder+file);
				image->setExifData(exifData);
				image->writeMetadata();
				
				try {
					std::filesystem::rename(tempFolder+file, outputFolder+file);
				} 
				catch (std::filesystem::filesystem_error& e) {
					std::cout << e.what() << '\n';
				}
			}
    		
    	};
    	
    	void gnssCallback(const sensor_msgs::NavSatFix& gnss){
    		
    		ROS_ERROR_STREAM("video_recorder_node gnss callBack");
    		
    		if(coord2Interpolate.size() < 2){
    			std::vector<double> currentCoord{gnss.longitude, gnss.latitude, gnss.altitude};
    			coord2Interpolate.push_back(currentCoord);
    		}
    		else{
    			std::vector<double> currentCoord{gnss.longitude, gnss.latitude, gnss.altitude};
    			coord2Interpolate.push_back(currentCoord);
    			coord2Interpolate.pop_front();
    			interpolateCoord();
    		}
    		
    		
    	};
    	//void imuCallback();
    	
    	

	private:
		std::string outputFolder;
		const std::string tempFolder = "/home/ubuntu/temp/";
		unsigned int count = 0;
		cv::VideoCapture cap;
		std::list<std::vector<double>> coord2Interpolate;
	
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
