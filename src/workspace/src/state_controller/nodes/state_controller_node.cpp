#include "ros/ros.h"
#include "state_controller/state_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stateControl");

  StateController stateControl;

  ros::spin();

  return 0;
}

