#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <cmath> // re. acos, sqrt //
#include "std_msgs/Float64.h"

#define PI 3.141592653589793238

class IKGenerator
{
  ros::Publisher column_publisher;
  ros::Publisher upper_arm_publisher;
  ros::Publisher lower_arm_publisher;
  ros::Subscriber subscriber;

  float column_height;
  float base_height;
  float base_width;
  float conveyor_height;
  float conveyor_width;
  float upper_arm_length;
  float lower_arm_length;

  public: IKGenerator(int rate)
  {
    ros::NodeHandle nh;

    this->subscriber=nh.subscribe("/chipsy_conveyor/joint_states",
      rate, &IKGenerator::sub_callback, this);

    this->column_publisher=nh.advertise<std_msgs::Float64>("/chipsy_arm/chipsy_arm/column_to_base_joint_position_controller/command", rate);
    this->upper_arm_publisher=nh.advertise<std_msgs::Float64>("/chipsy_arm/chipsy_arm/upper_arm_to_column_joint_position_controller/command", rate);
    this->lower_arm_publisher=nh.advertise<std_msgs::Float64>("/chipsy_arm/chipsy_arm/lower_arm_to_upper_arm_joint_position_controller/command", rate);

    nh.getParam("/chipsy_arm/column_length", column_height);
    nh.getParam("/chipsy_arm/base_height", base_height);
    nh.getParam("/chipsy_arm/base_width", base_width);
    nh.getParam("/chipsy_conveyor/conveyor_height", conveyor_height);
    nh.getParam("/chipsy_conveyor/conveyor_width", conveyor_width);
    nh.getParam("/chipsy_arm/upper_arm_length", upper_arm_length);
    nh.getParam("/chipsy_arm/lower_arm_length", lower_arm_length);
  }

  void sub_callback(const sensor_msgs::JointState::ConstPtr &jointstate)
  {
    std::vector<std_msgs::Float64> joint_commands=getIK(jointstate->position[0]);

    column_publisher.publish(joint_commands[0]);
    upper_arm_publisher.publish(joint_commands[1]);
    lower_arm_publisher.publish(joint_commands[2]);
  }

  std::vector<std_msgs::Float64> getIK(double position)
  {
    //joint_commands[0] has to be the angle of rotation of column
    //[1] has to be the angle of inclination of upper, [2] lower arm
    std::vector<std_msgs::Float64> joint_commands;
    std_msgs::Float64 push_msg;

    //push the angle of column
    push_msg.data=atan((double)(position-2.5)/(double)((base_width+conveyor_width)/2));
    joint_commands.push_back(push_msg);

    //gives raw distance from base of arm to middle of conveyor
    double distance_from_base=(double)((base_width+conveyor_width)*
      (base_width+conveyor_width))/4;
    distance_from_base+=conveyor_height*conveyor_height;
    distance_from_base+=(double)(position-2.5)*(double)(position-2.5);
    distance_from_base=sqrt(distance_from_base);

    //difference in height from top of column and conveyor
    double delta_height=base_height+column_height-conveyor_height;

    //distance of object from top of column
    double distance_from_column=(distance_from_base*distance_from_base)-
      (conveyor_height*conveyor_height);
    distance_from_column=sqrt(distance_from_column+(delta_height*delta_height));

    //Angle made by distance from column and column
    double upper_compensation_angle=acos(delta_height/distance_from_column);

    //base angle of triangle bw upper, lower, an distance from column
    //upper(cosx)+lower(cosx)=distance from column
    double temp_calc=distance_from_column/(upper_arm_length+lower_arm_length);
    temp_calc=(temp_calc>1.00d)?0.99:temp_calc;
    std::cout<<temp_calc<<std::endl;
    double base_angle = acos(temp_calc);

    //Final command for upper arm angle
    double upper_arm_angle=PI-base_angle-upper_compensation_angle;

    //push this upper arm angle
    push_msg.data=upper_arm_angle;
    joint_commands.push_back(push_msg);

    //Final command for lower arm angle
    double lower_arm_angle = 2*base_angle;

    //Push this lwer arm angle
    push_msg.data=lower_arm_angle;
    joint_commands.push_back(push_msg);

    return joint_commands;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chipsy_arm_controller");
  IKGenerator handler(1000);
  ros::spin();
  return 0;
}
