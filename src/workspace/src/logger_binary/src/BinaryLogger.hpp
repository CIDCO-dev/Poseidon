#pragma once

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include "../../utils/timestamp.h"
#include "types.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"

/* The BinaryLogger class creates a binary file with GPS, IMU and echosounder data */
class BinaryLogger{
public:
        BinaryLogger(std::string & logPath):outputFilePath(logPath){
                positionSubscriber = n.subscribe("fix", 1000, &BinaryLogger::processPositionCallback,this);
                attitudeSubscriber = n.subscribe("/imu/data", 1000, &BinaryLogger::processAttitudeCallback,this);
                depthSubscriber = n.subscribe("depth", 1000, &BinaryLogger::processDepthCallback,this);
				lidarSubscriber = n.subscribe("velodyne_points", 1000, &BinaryLogger::processLidarCallback, this);


                outputFileName = TimeUtils::getStringDate() + std::string(".log");

                outputFile.open(outputFilePath + "/" + outputFileName,std::ios::binary|std::ios::trunc);
				
				ROS_INFO("Logging binary data to %s", outputFilePath.c_str());
				
                if(!outputFile.good()){
                        throw std::invalid_argument("Couldn't open binary log file");
                }
        }

        ~BinaryLogger(){
                //close output file if necessary
                if(outputFile.is_open()) outputFile.close();
        }

        void processPositionCallback(const sensor_msgs::NavSatFix & msg){
                PacketHeader hdr;
                hdr.packetType=PACKET_POSITION;
                hdr.packetSize=sizeof(PositionPacket);
                hdr.packetTimestamp=TimeUtils::buildTimeStamp(msg.header.stamp.sec,msg.header.stamp.nsec);

                PositionPacket packet;

                packet.status=msg.status.status;
                packet.service=msg.status.service;
                packet.longitude=msg.longitude;
                packet.latitude=msg.latitude;
                packet.altitude=msg.altitude;
                memcpy(&packet.covariance,&msg.position_covariance,9*sizeof(double));
                packet.covarianceType=msg.position_covariance_type;

                outputFile.write((char*)&hdr,sizeof(PacketHeader));
                outputFile.write((char*)&packet,sizeof(PositionPacket));
        }

        void processAttitudeCallback(const sensor_msgs::Imu & msg){
                PacketHeader hdr;
                hdr.packetType=PACKET_ATTITUDE;
                hdr.packetSize=sizeof(AttitudePacket);
                hdr.packetTimestamp=TimeUtils::buildTimeStamp(msg.header.stamp.sec,msg.header.stamp.nsec);

                AttitudePacket packet;
                packet.orientation_w=msg.orientation.w;
                packet.orientation_x=msg.orientation.x;
                packet.orientation_y=msg.orientation.y;
                packet.orientation_z=msg.orientation.z;

                memcpy(&packet.orientation_covariance,&msg.orientation_covariance,9*sizeof(double));

                packet.angular_velocity_x=msg.angular_velocity.x;
                packet.angular_velocity_y=msg.angular_velocity.y;
                packet.angular_velocity_z=msg.angular_velocity.z;

                memcpy(&packet.angular_velocity_covariance,&msg.angular_velocity_covariance,9*sizeof(double));

                packet.linear_acceleration_x=msg.linear_acceleration.x;
                packet.linear_acceleration_y=msg.linear_acceleration.y;
                packet.linear_acceleration_z=msg.linear_acceleration.z;

                memcpy(&packet.linear_acceleration_covariance,&msg.linear_acceleration_covariance,9*sizeof(double));

                outputFile.write((char*)&hdr,sizeof(PacketHeader));
                outputFile.write((char*)&packet,sizeof(AttitudePacket));
        }

        void processDepthCallback(const geometry_msgs::PointStamped & msg){
                PacketHeader hdr;
                hdr.packetType=PACKET_DEPTH;
                hdr.packetSize=sizeof(DepthPacket);
                hdr.packetTimestamp=TimeUtils::buildTimeStamp(msg.header.stamp.sec,msg.header.stamp.nsec);

                DepthPacket packet;

                packet.depth_x=msg.point.x;
                packet.depth_y=msg.point.y;
                packet.depth_z=msg.point.z;

                outputFile.write((char*)&hdr,sizeof(PacketHeader));
                outputFile.write((char*)&packet,sizeof(DepthPacket));
        }
		
		void processLidarCallback(const sensor_msgs::PointCloud2& lidar){
			
			sensor_msgs::PointCloud lidarXYZ;
			sensor_msgs::convertPointCloud2ToPointCloud(lidar, lidarXYZ);
			
			std::vector<geometry_msgs::Point32> data = lidarXYZ.points;
			
			PacketHeader hdr;
            hdr.packetType=PACKET_LIDAR;
            hdr.packetSize=sizeof(LidarPacket) * data.size();
            hdr.packetTimestamp=TimeUtils::buildTimeStamp(lidar.header.stamp.sec,lidar.header.stamp.nsec);

            outputFile.write((char*)&hdr,sizeof(PacketHeader));
            
            for(auto const& point : data){
            	LidarPacket packet;
            	packet.laser_x = point.x;
            	packet.laser_y = point.y;
            	packet.laser_z = point.z;
            	
            	outputFile.write((char*)&packet, sizeof(LidarPacket));
            }
            
		}
		
        void run(){
                //Check for incoming messages @ 10Hz
                ros::Rate r(10.0);

                while(n.ok()){
                        ros::spinOnce();
                        r.sleep();
                }
        }

private:
        ros::NodeHandle n;
        ros::Subscriber positionSubscriber;
        ros::Subscriber attitudeSubscriber;
        ros::Subscriber depthSubscriber;
		ros::Subscriber lidarSubscriber;
		
        std::string  outputFilePath;
        std::string  outputFileName;
        std::ofstream outputFile;
};
