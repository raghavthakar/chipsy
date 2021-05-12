#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <cmath>


ros::Publisher pub;

void sub_callback(const sensor_msgs::JointState::ConstPtr &jointstate)
{
  std_msgs::Float64 msg;
  std::cout<<atan((jointstate->position[0]-2.5)/0.5)<<"\n";
  msg.data=atan((jointstate->position[0]-2.5)/0.5);
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  pub=nh.advertise<std_msgs::Float64>("/chipsy_arm/chipsy_arm/column_to_base_joint_position_controller/command", 1000);

  ros::Subscriber sub=nh.subscribe("/chipsy_conveyor/joint_states", 1000, sub_callback);

  float some_number;
  nh.getParam("/chipsy_arm/base_length", some_number);
  std::cout<<"Base length: "<<some_number<<"\n";

  ros::spin();
  return 0;
}
