#include "ros/ros.h"
#include "goal_planner/goal_planner.h"


int main(int argc,char** argv){

	ros::init(argc, argv, "goalPlanner");

	GoalPlanner goalPlanner;
	goalPlanner.run();
}


