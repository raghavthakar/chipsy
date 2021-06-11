# chipsy
Industrial Pick and Place robot. Chipsy uses geometric Inverse Kinematics to track a moving conveyor belt. Currently in development, the aim is to implement a fully autonomous robotic arm that tracks objects on a conveyor belt and places them on an adjacent UGV.

# Test It Out
Find the xacro description of the robot in `src/chipsy_description`.

`roslaunch chipsy_description chipsy_multibot.launch`

![Working animation](https://github.com/raghavthakar/chipsy/blob/main/ReadMe_Resources/chipsy_working.gif)

Make sure to source the workspace!

# Conveyor Belt Plugin For Gazebo-9
The repository also contains a custom plugin to simulate a coneyor belt in Gazebo-9 in `chipsy_plugins/src`.

`
