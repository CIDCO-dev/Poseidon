#ifndef DIAGNOISTICSTEST_H
#define DIAGNOISTICSTEST_H

#include <iostream>
#include <string>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

#include "ros/ros.h"

class DiagnosticsTest {
public:

	DiagnosticsTest(std::string name):key(name){}
	virtual ~DiagnosticsTest() {}
	
	virtual void do_test() = 0;
	
	virtual void to_json(rapidjson::Document &doc, rapidjson::Value &diagnosticsArray){
	
		rapidjson::Document diagnostic(rapidjson::kObjectType);
		
		rapidjson::Value diagName(this->key.c_str(), doc.GetAllocator());
		diagnostic.AddMember("name", diagName, doc.GetAllocator());
		
		rapidjson::Value infoStr(this->value.c_str(), doc.GetAllocator());
		diagnostic.AddMember("message", infoStr, doc.GetAllocator());
		
		rapidjson::Value status(this->status);
		diagnostic.AddMember("status", status, doc.GetAllocator());

		diagnosticsArray.PushBack(diagnostic, doc.GetAllocator());
	}

protected:
	std::string key;
	std::string value = "";
	bool status = false;
};
#endif
