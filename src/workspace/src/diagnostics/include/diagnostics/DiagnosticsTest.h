#ifndef DIAGNOISTICSTEST_H
#define DIAGNOISTICSTEST_H

#include <iostream>
#include <string>
#include<chrono>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
/*#include "sensor_msgs/Imu.h"*/
/*#include "sensor_msgs/PointCloud2.h"*/
/*#include "sensor_msgs/point_cloud_conversion.h"*/
/*#include "sensor_msgs/PointCloud.h"*/
/*#include "geometry_msgs/PointStamped.h"*/
/*#include "geometry_msgs/TransformStamped.h"*/
/*#include "nav_msgs/Odometry.h"*/
/*#include "std_msgs/String.h"*/
/*#include <tf2_ros/transform_listener.h>*/
/*#include "tf2_geometry_msgs/tf2_geometry_msgs.h"*/

class DiagnosticsTest {
public:
	DiagnosticsTest(){}

	virtual ~DiagnosticsTest() {}

	virtual bool do_test() = 0;
	
	virtual void to_json(rapidjson::Document &doc, rapidjson::Value &diagnosticsArray){
	
		rapidjson::Document diagnostic(rapidjson::kObjectType);
		
		rapidjson::Value diagName(this->key.c_str(), diagnostic.GetAllocator());
		diagnostic.AddMember("name", diagName, diagnostic.GetAllocator());
		
		rapidjson::Value infoStr(this->value.c_str(), diagnostic.GetAllocator());
		diagnostic.AddMember("message", infoStr, diagnostic.GetAllocator());
		
		rapidjson::Value status(this->status);
		diagnostic.AddMember("status", status, diagnostic.GetAllocator());

		diagnosticsArray.PushBack(diagnostic, doc.GetAllocator());
	}

protected:
	std::string key;
	std::string value;
	bool status = false;
};
#endif
