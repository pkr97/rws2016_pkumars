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
	#include "std_msgs/Float32MultiArray.h"
	#include <geometry_msgs/PoseStamped.h>
	
	// publisher defined
	ros::Publisher kinectEstimeted; // publish end-effector
	
	
void callback(const std_msgs::Float32MultiArray& signal)
{
		// publisher definition
		geometry_msgs::PoseStamped msg;
	
		// KinectEstimated (KE) header stamp and frame id
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/base1";
	
		// KE Position 
		msg.pose.position.x = 0.1; 
		msg.pose.position.y = 0.1; 
		msg.pose.position.z = 0.1;  
		// KE Orientation 
		msg.pose.orientation.x = 0.0; 
		msg.pose.orientation.y = 0.0; 
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 1.0;   

		// EE Publish 
		kinectEstimeted.publish(msg);

}
	int main(int argc, char** argv)
{
	ros::init(argc, argv, "estimation_node");
	
	ros::NodeHandle nh;
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/ballCord", 1, callback);
	// publish
	kinectEstimeted = nh.advertise<geometry_msgs::PoseStamped>("/kinectEstimeted", 1);
	
	
	ros::spin();
	

	return 0;
}

