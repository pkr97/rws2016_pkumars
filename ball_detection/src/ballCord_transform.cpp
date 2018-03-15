/**************************************************************************************************
*	Name:           ballCord_transform.cpp														  *
*	Revision:		No revision				                                                      *
*	Date:           10/05/2016													                  *
*	Author:         Prasanna Kumar Routray											              *
*	Comments:       for the purpose of coordinate transformation from kinect frame to robot frame *
*	images																			              *
*	Libraries:																		              *
*	Notes:          Code generated under Ubuntu 14.04 using ROS indigo				              *
*   Subscription:- 	Estimated landing point of ball at a given height in KINECT frame			  *
* 	Publish:-		Estimated landing point of ball at a given height in ROBOT frame			  *
*	Compilation:	catkin_make														              *
**************************************************************************************************/	

// c++ headers
#include <iostream>
// ROS headers
#include <ros/ros.h>
// pcl headers
#include <pcl/console/print.h>
#include <pcl/common/time.h> // for watch(timer)
#include <geometry_msgs/PoseStamped.h>

// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
using namespace std;

/**
 *  It should be noted that the coordinates are the distances from respective frame.
 *  Thses distances are in "meters".
 */

ros::Publisher robotEstimeted; // publish end-effector

void callback(const geometry_msgs::PoseStampedConstPtr& signal)
{	
	pcl::StopWatch watch;
	
	double kinectX = signal->pose.position.x;
	double kinectY = signal->pose.position.y;
	double kinectZ = signal->pose.position.z;
	
	double xOffset, yOffset, zOffset;
	
	// in meters
	xOffset =  0.00;
	yOffset = -0.06; 
	zOffset = -0.05;
	
	// publisher definition
	geometry_msgs::PoseStamped robotEE;
	
	// EE header stamp and frame id
	robotEE.header.stamp = ros::Time::now();
	robotEE.header.frame_id = "/base1";
	
	// EE Position 
	robotEE.pose.position.x =  kinectX + xOffset; 
	robotEE.pose.position.y = -kinectZ + xOffset; 
	robotEE.pose.position.z =  kinectY + xOffset;
	
	// EE Orientation 
	robotEE.pose.orientation.x = 1.0; 
	robotEE.pose.orientation.y = 0.0; 
	robotEE.pose.orientation.z = 0.0;
	robotEE.pose.orientation.w = 0.0;   
	
	cout<<"x--> "<<robotEE.pose.position.x<<"  y--> "<<robotEE.pose.position.y<<"  z--> "<<robotEE.pose.position.z<<endl;
	
	// EE Publish 
	robotEstimeted.publish(robotEE);

	pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_node");
	ros::NodeHandle nh;
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/kinectEstimeted", 1, callback);
	robotEstimeted = nh.advertise<geometry_msgs::PoseStamped>("/robotEstimeted", 1);

	ros::spin();
	return 0;
}

