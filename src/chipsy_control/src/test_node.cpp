#include "ros/ros.h"
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematics");
  ros::NodeHandle nh;
  std::cout<<"Running!\n";
  ros::spin();
  return 0;
}
