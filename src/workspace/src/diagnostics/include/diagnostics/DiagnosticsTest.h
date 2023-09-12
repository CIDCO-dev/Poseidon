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
		
		rapidjson::Value diagnostic;
		diagnostic.SetObject();
		rapidjson::Value diagnosticArray(rapidjson::kArrayType);
		
		rapidjson::Value info(rapidjson::kObjectType);
		rapidjson::Value infoStr(this->value.c_str(), doc.GetAllocator());
		info.AddMember("info", infoStr, doc.GetAllocator());
		diagnosticArray.PushBack(info, doc.GetAllocator());
		
		rapidjson::Value status(rapidjson::kObjectType);
		rapidjson::Value statusValue(this->status);
		status.AddMember("status", statusValue, doc.GetAllocator());
		diagnosticArray.PushBack(status, doc.GetAllocator());
		
		//diagnosticsArray.PushBack(diagnostic, doc.GetAllocator());

		rapidjson::Value keyVal;
		keyVal.SetString(this->key.c_str(), this->key.length(), doc.GetAllocator());
		diagnostic.AddMember(keyVal, diagnosticArray, doc.GetAllocator());
		diagnosticsArray.PushBack(diagnostic, doc.GetAllocator());
	}

protected:
	std::string key;
	std::string value;
	bool status = false;
};
#endif
