#!usr/bin/env/python
import rospy
import os

if __name__=="__main__":
    try:
        os.system("rosrun gazebo_ros spawn_model -sdf -file"+" test_sdf.sdf -model test_sdf -x 5 -y 5")
    except:
        print("bruh")
