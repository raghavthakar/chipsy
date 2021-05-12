#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <cmath>

class IKGenerator
{
  ros::Publisher publisher;
  ros::Subscriber subscriber;

  public: IKGenerator(int rate)
  {
    ros::NodeHandle nh;
    this->subscriber=nh.subscribe("/chipsy_conveyor/joint_states", rate, &IKGenerator::sub_callback, this);
  }

  void sub_callback(const sensor_msgs::JointState::ConstPtr &jointstate)
  {
    std::cout<<atan((jointstate->position[0]-2.5)/0.5)<<"\n";
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chipsy_arm_controller");
  IKGenerator handler(1000);
  ros::spin();
  return 0;
}
