/***********************************************************************************
Name:           ball17.cpp
Revision:		revision of ball detection and tracking algorithm
Date:           14-03-2016
Author:         Prasanna Kumar Routray
Comments:       for the purpose of ball catching analysis process 
images
Revision:		revision of ball16.cpp (trajectory stuff added)
Libraries:
Notes:          Code generated under Ubuntu using ROS, OpenCV, PCL
Compilation:	catkin_make
***********************************************************************************/
	// ROS headers
	#include <ros/ros.h>
	// c++ headers
	#include <iostream>
	// Message publication
	#include <geometry_msgs/PoseStamped.h>
	
	// publisher defined
	ros::Publisher kinectEstimeted; // publish end-effector
	
void callback(const geometry_msgs::PoseStampedConstPtr& signal)
{
	
}
	int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_node");
	
	ros::NodeHandle nh;
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/kinectEstimeted", 1, callback);

	ros::spin();
	return 0;
}

