# rr_openrover_java
Provides a way to control an OpenRover robot using ROS and the OpenRover JavaSDK

## Prerequisites
-OpenRover JavaSDK 2.0.02 or above  
-ROS Kinetic or above  
-Ubuntu 16.04 or above  

## Install 
cd ~/catkin_ws/src  
git clone https://github.com/RoverRobotics/rr_openrover_java.git  
cd ~/catkin_ws  
catkin_make  
source /devel/setup.bash  

## Running Examples
roslaunch rr_openrover_java avatar_startup.launch  
roslaunch rr_openrover_java joystick_example.launch  

## Published Topics:

* `/avatar/battery/cell1/soc`:
  Publishes `std_msgs/Int32`, battery cell 1 state of charge as a percentage [1 to 100]

* `/avatar/battery/cell2/soc`:
  Publishes `std_msgs/Int32`, battery cell 2 state of charge as a percentage [1 to 100]

* `/avatar/charging`:
  Publishes `std_msgs/Int32`, not very robust yet yet. This topic publishes charge state of the robot 

* `/avatar/enc`:
  Publishes `geometry_msgs/Vector3Stamped` encoder feedback in [counts] 

* `/avatar/motor1/temp`:
  Publishes `std_msgs/Int32` motor 1 temp in [C]

* `/avatar/motor2/temp`:
  Publishes `std_msgs/Int32` motor 2 temp in [C]

* `/avatar/odom`:
  Publishes `nav_msgs/Odometry` encoder feedback in [m/s] and [rad/s], usefull as odometry source for kalman filter. Not accurate yet.

* `/avatar/odom_diff`:
  Publishes `geometry_msgs/Vector3Stamped` provides the difference in commanded and measured odometry

* `/avatar/status`:
  Publishes `std_msgs/String` un-parsed string used for publishing the other topics


## Subcribed Topics:

* `/led`:
  Subscribes to `std_msgs/Int32` to turn the LED on and off

* `/cmd_vel`:
  Subscribes to `geometry_msgs/Twist` used to command the wheels. linear.x is used to move forward and back, angular.z is used to turn left and right. 

* `/avatar/charging_override`:
  Subscribes to `std_msgs/Int32` since /avatar/charging is not robust this provides a way to manually change the /avatar/charging topic. This is useful if the topic is used in a state machine

## Parameters:

* `ip_address` : IP Address of SwiftNav Piksi, use the SwiftNAv console to check what this is set to 

* `tcp_port`: The tcp port of SwiftNav Piksi, use the SwiftNav console to check what this is set to
