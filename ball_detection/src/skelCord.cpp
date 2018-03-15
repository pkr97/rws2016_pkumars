#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float32MultiArray.h"

//// c++ headers
#include <iostream>
// ROS headers
#include <ros/ros.h>
#include <sys/time.h>
#include <chrono>


ros::Publisher skelData;


long double getTime()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = 
        boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();
    
	long double milliseconds = td.total_milliseconds();
	
	return milliseconds;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_skeleton_tf_listener");
	
	ros::NodeHandle node;
/*
	// publisher declaration
	ros::Publisher neck_joint = node.advertise<geometry_msgs::PoseStamped>("/neck_joint", 1);
	ros::Publisher head_joint = node.advertise<geometry_msgs::PoseStamped>("/head_joint", 1);
	ros::Publisher torso_joint = node.advertise<geometry_msgs::PoseStamped>("/torso_joint", 1);
	ros::Publisher left_shoulder_joint = node.advertise<geometry_msgs::PoseStamped>("/left_shoulder_joint", 1);
	ros::Publisher left_elbow_joint = node.advertise<geometry_msgs::PoseStamped>("/left_elbow_joint", 1);
	ros::Publisher left_hand_joint = node.advertise<geometry_msgs::PoseStamped>("/left_hand_joint", 1);
	ros::Publisher right_shoulder_joint = node.advertise<geometry_msgs::PoseStamped>("/right_shoulder_joint", 1);
	ros::Publisher right_elbow_joint = node.advertise<geometry_msgs::PoseStamped>("/right_elbow_joint", 1);
	ros::Publisher right_hand_joint = node.advertise<geometry_msgs::PoseStamped>("/right_hand_joint", 1);
	ros::Publisher left_hip_joint = node.advertise<geometry_msgs::PoseStamped>("/left_hip_joint", 1);
	ros::Publisher left_knee_joint = node.advertise<geometry_msgs::PoseStamped>("/left_knee_joint", 1);
	ros::Publisher left_foot_joint = node.advertise<geometry_msgs::PoseStamped>("/left_foot_joint", 1);
	ros::Publisher right_hip_joint = node.advertise<geometry_msgs::PoseStamped>("/right_hip_joint", 1);
	ros::Publisher right_knee_joint = node.advertise<geometry_msgs::PoseStamped>("/right_knee_joint", 1);
	ros::Publisher right_foot_joint = node.advertise<geometry_msgs::PoseStamped>("/right_foot_joint", 1);
*/
	
	skelData = node.advertise<std_msgs::Float32MultiArray>("/skelData", 1);
	
	// listener 
	tf::TransformListener listener;
	
	ros::Rate rate(30.0); // frequency of operation
	
	while (node.ok())
	{
		// Transforms declared for each joint
		tf::StampedTransform transform_neck, transform_head, transform_torso, 
								transform_left_shoulder, transform_left_elbow, transform_left_hand, 
									transform_right_shoulder, transform_right_elbow, transform_right_hand, 
										transform_left_hip, transform_left_knee, transform_left_foot,
											transform_right_hip, transform_right_knee, transform_right_foot;
		try
		{
			// each joint frame to reference frame transforms 
			listener.lookupTransform("/neck_1", "/openni_depth_frame",ros::Time(0), transform_neck);
			listener.lookupTransform("/head_1", "/openni_depth_frame",ros::Time(0), transform_head);
			listener.lookupTransform("/torso_1", "/openni_depth_frame",ros::Time(0), transform_torso);
			listener.lookupTransform("/left_shoulder_1", "/openni_depth_frame",ros::Time(0), transform_left_shoulder);
			listener.lookupTransform("/left_elbow_1", "/openni_depth_frame",ros::Time(0), transform_left_elbow);
			listener.lookupTransform("/left_hand_1", "/openni_depth_frame",ros::Time(0), transform_left_hand);
			listener.lookupTransform("/right_shoulder_1", "/openni_depth_frame",ros::Time(0), transform_right_shoulder);
			listener.lookupTransform("/right_elbow_1", "/openni_depth_frame",ros::Time(0), transform_right_elbow);
			listener.lookupTransform("/right_hand_1", "/openni_depth_frame",ros::Time(0), transform_right_hand);
			listener.lookupTransform("/left_hip_1", "/openni_depth_frame",ros::Time(0), transform_left_hip);
			listener.lookupTransform("/left_knee_1", "/openni_depth_frame",ros::Time(0), transform_left_knee);
			listener.lookupTransform("/left_foot_1", "/openni_depth_frame",ros::Time(0), transform_left_foot);
			listener.lookupTransform("/right_hip_1", "/openni_depth_frame",ros::Time(0), transform_right_hip);
			listener.lookupTransform("/right_knee_1", "/openni_depth_frame",ros::Time(0), transform_right_knee);
			listener.lookupTransform("/right_foot_1", "/openni_depth_frame",ros::Time(0), transform_right_foot);
			
		}
			catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(0.01).sleep();
			continue;
		}
		
		// publisher definition
		std_msgs::Float32MultiArray dataArray;
		dataArray.data.clear();
/*	
		// geometry points declaration for storing 3D coordinates of joints and then published later 
		geometry_msgs::PoseStamped neck_pose, head_pose, torso_pose, 
									left_shoulder_pose, left_elbow_pose, left_hand_pose,
									right_shoulder_pose, right_elbow_pose, right_hand_pose, 
										left_hip_pose, left_knee_pose, left_foot_pose, 
											right_hip_pose, right_knee_pose, right_foot_pose;
		
		// EE header stamp and frame id
		neck_pose.header.stamp = ros::Time::now();
		head_pose.header.stamp = ros::Time::now();
		torso_pose.header.stamp = ros::Time::now();
		left_shoulder_pose.header.stamp = ros::Time::now();
		left_elbow_pose.header.stamp = ros::Time::now();
		left_hand_pose.header.stamp = ros::Time::now();
		right_shoulder_pose.header.stamp = ros::Time::now();
		right_elbow_pose.header.stamp = ros::Time::now();
		right_hand_pose.header.stamp = ros::Time::now();
		left_hip_pose.header.stamp = ros::Time::now();
		left_knee_pose.header.stamp = ros::Time::now();
		left_foot_pose.header.stamp = ros::Time::now();
		right_hip_pose.header.stamp = ros::Time::now();
		right_knee_pose.header.stamp = ros::Time::now();
		right_foot_pose.header.stamp = ros::Time::now();

		// EE header stamp and frame id
		neck_pose.header.frame_id = "/pose1";
		head_pose.header.frame_id = "/pose2";
		torso_pose.header.frame_id = "/pose3";
		left_shoulder_pose.header.frame_id = "/pose4";
		left_elbow_pose.header.frame_id = "/pose5";
		left_hand_pose.header.frame_id = "/pose6";
		right_shoulder_pose.header.frame_id = "/pose7";
		right_elbow_pose.header.frame_id = "/pose8";
		right_hand_pose.header.frame_id = "/pose9";
		left_hip_pose.header.frame_id = "/pose10";
		left_knee_pose.header.frame_id = "/pose11";
		left_foot_pose.header.frame_id = "/pose12";
		right_hip_pose.header.frame_id = "/pose13";
		right_knee_pose.header.frame_id = "/pose14";
		right_foot_pose.header.frame_id = "/pose15";
		
		// joint position extraction and store
		// neck joint									
		// position
		neck_pose.pose.position.x = transform_neck.getOrigin().x(); 
		neck_pose.pose.position.y = transform_neck.getOrigin().y(); 
		neck_pose.pose.position.z = transform_neck.getOrigin().z();  
		// orientation
		neck_pose.pose.orientation.x = transform_neck.getRotation().x(); 
		neck_pose.pose.orientation.y = transform_neck.getRotation().y(); 
		neck_pose.pose.orientation.z = transform_neck.getRotation().z(); 
		neck_pose.pose.orientation.w = transform_neck.getRotation().w();  
		
		// head joint
		// position
		head_pose.pose.position.x = transform_head.getOrigin().x(); 
		head_pose.pose.position.y = transform_head.getOrigin().y(); 
		head_pose.pose.position.z = transform_head.getOrigin().z();  
		// orientation
		head_pose.pose.orientation.x = transform_head.getRotation().x(); 
		head_pose.pose.orientation.y = transform_head.getRotation().y(); 
		head_pose.pose.orientation.z = transform_head.getRotation().z(); 
		head_pose.pose.orientation.w = transform_head.getRotation().w();  

		// torso joint
		// position
		torso_pose.pose.position.x = transform_torso.getOrigin().x(); 
		torso_pose.pose.position.y = transform_torso.getOrigin().y(); 
		torso_pose.pose.position.z = transform_torso.getOrigin().z();  
		// orientation
		torso_pose.pose.orientation.x = transform_torso.getRotation().x(); 
		torso_pose.pose.orientation.y = transform_torso.getRotation().y(); 
		torso_pose.pose.orientation.z = transform_torso.getRotation().z(); 
		torso_pose.pose.orientation.w = transform_torso.getRotation().w();  

		// left shoulder joint
		// position
		left_shoulder_pose.pose.position.x = transform_left_shoulder.getOrigin().x(); 
		left_shoulder_pose.pose.position.y = transform_left_shoulder.getOrigin().y(); 
		left_shoulder_pose.pose.position.z = transform_left_shoulder.getOrigin().z();  
		// orientation
		left_shoulder_pose.pose.orientation.x = transform_left_shoulder.getRotation().x(); 
		left_shoulder_pose.pose.orientation.y = transform_left_shoulder.getRotation().y(); 
		left_shoulder_pose.pose.orientation.z = transform_left_shoulder.getRotation().z(); 
		left_shoulder_pose.pose.orientation.w = transform_left_shoulder.getRotation().w();  
		
		// left elbow joint
		// position
		left_elbow_pose.pose.position.x = transform_left_elbow.getOrigin().x(); 
		left_elbow_pose.pose.position.y = transform_left_elbow.getOrigin().y(); 
		left_elbow_pose.pose.position.z = transform_left_elbow.getOrigin().z();  
		// orientation
		left_elbow_pose.pose.orientation.x = transform_left_elbow.getRotation().x(); 
		left_elbow_pose.pose.orientation.y = transform_left_elbow.getRotation().y(); 
		left_elbow_pose.pose.orientation.z = transform_left_elbow.getRotation().z(); 
		left_elbow_pose.pose.orientation.w = transform_left_elbow.getRotation().w();  

		// left hand joint
		// position
		left_hand_pose.pose.position.x = transform_left_hand.getOrigin().x(); 
		left_hand_pose.pose.position.y = transform_left_hand.getOrigin().y(); 
		left_hand_pose.pose.position.z = transform_left_hand.getOrigin().z();  
		// orientation
		left_hand_pose.pose.orientation.x = transform_left_hand.getRotation().x(); 
		left_hand_pose.pose.orientation.y = transform_left_hand.getRotation().y(); 
		left_hand_pose.pose.orientation.z = transform_left_hand.getRotation().z(); 
		left_hand_pose.pose.orientation.w = transform_left_hand.getRotation().w();  

		// right shoulder joint
		// position
		right_shoulder_pose.pose.position.x = transform_right_shoulder.getOrigin().x(); 
		right_shoulder_pose.pose.position.y = transform_right_shoulder.getOrigin().y(); 
		right_shoulder_pose.pose.position.z = transform_right_shoulder.getOrigin().z();  
		// orientation
		right_shoulder_pose.pose.orientation.x = transform_right_shoulder.getRotation().x(); 
		right_shoulder_pose.pose.orientation.y = transform_right_shoulder.getRotation().y(); 
		right_shoulder_pose.pose.orientation.z = transform_right_shoulder.getRotation().z(); 
		right_shoulder_pose.pose.orientation.w = transform_right_shoulder.getRotation().w();  

		// right elbow joint
		// position
		right_elbow_pose.pose.position.x = transform_right_elbow.getOrigin().x(); 
		right_elbow_pose.pose.position.y = transform_right_elbow.getOrigin().y(); 
		right_elbow_pose.pose.position.z = transform_right_elbow.getOrigin().z();  
		// orientation
		right_elbow_pose.pose.orientation.x = transform_right_elbow.getRotation().x(); 
		right_elbow_pose.pose.orientation.y = transform_right_elbow.getRotation().y(); 
		right_elbow_pose.pose.orientation.z = transform_right_elbow.getRotation().z(); 
		right_elbow_pose.pose.orientation.w = transform_right_elbow.getRotation().w();  

		// right hand joint
		// position
		right_hand_pose.pose.position.x = transform_right_hand.getOrigin().x(); 
		right_hand_pose.pose.position.y = transform_right_hand.getOrigin().y(); 
		right_hand_pose.pose.position.z = transform_right_hand.getOrigin().z();  
		// orientation
		right_hand_pose.pose.orientation.x = transform_right_hand.getRotation().x(); 
		right_hand_pose.pose.orientation.y = transform_right_hand.getRotation().y(); 
		right_hand_pose.pose.orientation.z = transform_right_hand.getRotation().z(); 
		right_hand_pose.pose.orientation.w = transform_right_hand.getRotation().w();  

		// left hip joint
		// position
		left_hip_pose.pose.position.x = transform_left_hip.getOrigin().x(); 
		left_hip_pose.pose.position.y = transform_left_hip.getOrigin().y(); 
		left_hip_pose.pose.position.z = transform_left_hip.getOrigin().z();  
		// orientation
		left_hip_pose.pose.orientation.x = transform_left_hip.getRotation().x(); 
		left_hip_pose.pose.orientation.y = transform_left_hip.getRotation().y(); 
		left_hip_pose.pose.orientation.z = transform_left_hip.getRotation().z(); 
		left_hip_pose.pose.orientation.w = transform_left_hip.getRotation().w();  

		// left knee joint
		// position
		left_knee_pose.pose.position.x = transform_left_knee.getOrigin().x(); 
		left_knee_pose.pose.position.y = transform_left_knee.getOrigin().y(); 
		left_knee_pose.pose.position.z = transform_left_knee.getOrigin().z();  
		// orientation
		left_knee_pose.pose.orientation.x = transform_left_knee.getRotation().x(); 
		left_knee_pose.pose.orientation.y = transform_left_knee.getRotation().y(); 
		left_knee_pose.pose.orientation.z = transform_left_knee.getRotation().z(); 
		left_knee_pose.pose.orientation.w = transform_left_knee.getRotation().w();  
		
		// left foot joint
		// position
		left_foot_pose.pose.position.x = transform_left_foot.getOrigin().x(); 
		left_foot_pose.pose.position.y = transform_left_foot.getOrigin().y(); 
		left_foot_pose.pose.position.z = transform_left_foot.getOrigin().z();  
		// orientation
		left_foot_pose.pose.orientation.x = transform_left_foot.getRotation().x(); 
		left_foot_pose.pose.orientation.y = transform_left_foot.getRotation().y(); 
		left_foot_pose.pose.orientation.z = transform_left_foot.getRotation().z(); 
		left_foot_pose.pose.orientation.w = transform_left_foot.getRotation().w();  

		// right hip joint
		// position
		right_hip_pose.pose.position.x = transform_right_hip.getOrigin().x(); 
		right_hip_pose.pose.position.y = transform_right_hip.getOrigin().y(); 
		right_hip_pose.pose.position.z = transform_right_hip.getOrigin().z();  
		// orientation
		right_hip_pose.pose.orientation.x = transform_right_hip.getRotation().x(); 
		right_hip_pose.pose.orientation.y = transform_right_hip.getRotation().y(); 
		right_hip_pose.pose.orientation.z = transform_right_hip.getRotation().z(); 
		right_hip_pose.pose.orientation.w = transform_right_hip.getRotation().w();  

		// right knee joint
		// position
		right_knee_pose.pose.position.x = transform_right_knee.getOrigin().x(); 
		right_knee_pose.pose.position.y = transform_right_knee.getOrigin().y(); 
		right_knee_pose.pose.position.z = transform_right_knee.getOrigin().z();  
		// orientation
		right_knee_pose.pose.orientation.x = transform_right_knee.getRotation().x(); 
		right_knee_pose.pose.orientation.y = transform_right_knee.getRotation().y(); 
		right_knee_pose.pose.orientation.z = transform_right_knee.getRotation().z(); 
		right_knee_pose.pose.orientation.w = transform_right_knee.getRotation().w();  

		// right foot joint
		// position
		right_foot_pose.pose.position.x = transform_right_foot.getOrigin().x(); 
		right_foot_pose.pose.position.y = transform_right_foot.getOrigin().y(); 
		right_foot_pose.pose.position.z = transform_right_foot.getOrigin().z();  
		// orientation
		right_foot_pose.pose.orientation.x = transform_right_foot.getRotation().x(); 
		right_foot_pose.pose.orientation.y = transform_right_foot.getRotation().y(); 
		right_foot_pose.pose.orientation.z = transform_right_foot.getRotation().z(); 
		right_foot_pose.pose.orientation.w = transform_right_foot.getRotation().w();  

		// joint pose publish
		neck_joint.publish(neck_pose);
		head_joint.publish(head_pose);
		torso_joint.publish(torso_pose);
		left_shoulder_joint.publish(left_shoulder_pose);
		left_elbow_joint.publish(left_elbow_pose);
		left_hand_joint.publish(left_hand_pose);
		right_shoulder_joint.publish(right_shoulder_pose);
		right_elbow_joint.publish(right_elbow_pose);
		right_hand_joint.publish(right_hand_pose);
		left_hip_joint.publish(left_hip_pose);
		left_knee_joint.publish(left_knee_pose);
		left_foot_joint.publish(left_foot_pose);
		right_hip_joint.publish(right_hip_pose);
		right_knee_joint.publish(right_knee_pose);
		right_foot_joint.publish(right_foot_pose);
*/

		double dtInstant = getTime();
		dataArray.data.push_back(dtInstant);
		
		// neck joint
		// position
		dataArray.data.push_back(transform_neck.getOrigin().x());
		dataArray.data.push_back(transform_neck.getOrigin().y());
		dataArray.data.push_back(transform_neck.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_neck.getRotation().x());
		dataArray.data.push_back(transform_neck.getRotation().y());
		dataArray.data.push_back(transform_neck.getRotation().z());
		dataArray.data.push_back(transform_neck.getRotation().w());*/

		// head joint
		// position
		dataArray.data.push_back(transform_head.getOrigin().x());
		dataArray.data.push_back(transform_head.getOrigin().y());
		dataArray.data.push_back(transform_head.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_head.getRotation().x()); 
		dataArray.data.push_back(transform_head.getRotation().y()); 
		dataArray.data.push_back(transform_head.getRotation().z()); 
		dataArray.data.push_back(transform_head.getRotation().w());*/

		// torso joint
		// position
		dataArray.data.push_back(transform_torso.getOrigin().x());
		dataArray.data.push_back(transform_torso.getOrigin().y());
		dataArray.data.push_back(transform_torso.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_torso.getRotation().x());
		dataArray.data.push_back(transform_torso.getRotation().y());
		dataArray.data.push_back(transform_torso.getRotation().z());
		dataArray.data.push_back(transform_torso.getRotation().w());*/

		// left shoulder joint
		// position
		dataArray.data.push_back(transform_left_shoulder.getOrigin().x()); 
		dataArray.data.push_back(transform_left_shoulder.getOrigin().y()); 
		dataArray.data.push_back(transform_left_shoulder.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_left_shoulder.getRotation().x());
		dataArray.data.push_back(transform_left_shoulder.getRotation().y());
		dataArray.data.push_back(transform_left_shoulder.getRotation().z());
		dataArray.data.push_back(transform_left_shoulder.getRotation().w());*/
				
		// left elbow joint
		// position
		dataArray.data.push_back(transform_left_elbow.getOrigin().x()); 
		dataArray.data.push_back(transform_left_elbow.getOrigin().y()); 
		dataArray.data.push_back(transform_left_elbow.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_left_elbow.getRotation().x()); 
		dataArray.data.push_back(transform_left_elbow.getRotation().y()); 
		dataArray.data.push_back(transform_left_elbow.getRotation().z()); 
		dataArray.data.push_back(transform_left_elbow.getRotation().w());*/

		// left hand joint
		// position
		dataArray.data.push_back(transform_left_hand.getOrigin().x()); 
		dataArray.data.push_back(transform_left_hand.getOrigin().y()); 
		dataArray.data.push_back(transform_left_hand.getOrigin().z());
		// orientation
/*		dataArray.data.push_back(transform_left_hand.getRotation().x()); 
		dataArray.data.push_back(transform_left_hand.getRotation().y()); 
		dataArray.data.push_back(transform_left_hand.getRotation().z()); 
		dataArray.data.push_back(transform_left_hand.getRotation().w());*/

		// right shoulder joint
		// position
		dataArray.data.push_back(transform_right_shoulder.getOrigin().x()); 
		dataArray.data.push_back(transform_right_shoulder.getOrigin().y()); 
		dataArray.data.push_back(transform_right_shoulder.getOrigin().z());
		// orientation
/*		dataArray.data.push_back(transform_right_shoulder.getRotation().x()); 
		dataArray.data.push_back(transform_right_shoulder.getRotation().y()); 
		dataArray.data.push_back(transform_right_shoulder.getRotation().z()); 
		dataArray.data.push_back(transform_right_shoulder.getRotation().w());*/

		// right elbow joint
		// position
		dataArray.data.push_back(transform_right_elbow.getOrigin().x()); 
		dataArray.data.push_back(transform_right_elbow.getOrigin().y()); 
		dataArray.data.push_back(transform_right_elbow.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_right_elbow.getRotation().x()); 
		dataArray.data.push_back(transform_right_elbow.getRotation().y()); 
		dataArray.data.push_back(transform_right_elbow.getRotation().z()); 
		dataArray.data.push_back(transform_right_elbow.getRotation().w());*/

		// right hand joint
		// position
		dataArray.data.push_back(transform_right_hand.getOrigin().x()); 
		dataArray.data.push_back(transform_right_hand.getOrigin().y()); 
		dataArray.data.push_back(transform_right_hand.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_right_hand.getRotation().x()); 
		dataArray.data.push_back(transform_right_hand.getRotation().y()); 
		dataArray.data.push_back(transform_right_hand.getRotation().z()); 
		dataArray.data.push_back(transform_right_hand.getRotation().w());*/

		// left hip joint
		// position
		dataArray.data.push_back(transform_left_hip.getOrigin().x()); 
		dataArray.data.push_back(transform_left_hip.getOrigin().y()); 
		dataArray.data.push_back(transform_left_hip.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_left_hip.getRotation().x()); 
		dataArray.data.push_back(transform_left_hip.getRotation().y()); 
		dataArray.data.push_back(transform_left_hip.getRotation().z()); 
		dataArray.data.push_back(transform_left_hip.getRotation().w());*/

		// left knee joint
		// position
		dataArray.data.push_back(transform_left_knee.getOrigin().x()); 
		dataArray.data.push_back(transform_left_knee.getOrigin().y()); 
		dataArray.data.push_back(transform_left_knee.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_left_knee.getRotation().x()); 
		dataArray.data.push_back(transform_left_knee.getRotation().y()); 
		dataArray.data.push_back(transform_left_knee.getRotation().z()); 
		dataArray.data.push_back(transform_left_knee.getRotation().w());*/
		
		// left foot joint
		// position
		dataArray.data.push_back(transform_left_foot.getOrigin().x()); 
		dataArray.data.push_back(transform_left_foot.getOrigin().y()); 
		dataArray.data.push_back(transform_left_foot.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_left_foot.getRotation().x()); 
		dataArray.data.push_back(transform_left_foot.getRotation().y()); 
		dataArray.data.push_back(transform_left_foot.getRotation().z()); 
		dataArray.data.push_back(transform_left_foot.getRotation().w());*/
		// right hip joint
		// position
		dataArray.data.push_back(transform_right_hip.getOrigin().x()); 
		dataArray.data.push_back(transform_right_hip.getOrigin().y()); 
		dataArray.data.push_back(transform_right_hip.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_right_hip.getRotation().x()); 
		dataArray.data.push_back(transform_right_hip.getRotation().y()); 
		dataArray.data.push_back(transform_right_hip.getRotation().z()); 
		dataArray.data.push_back(transform_right_hip.getRotation().w());*/

		// right knee joint
		// position
		dataArray.data.push_back(transform_right_knee.getOrigin().x()); 
		dataArray.data.push_back(transform_right_knee.getOrigin().y()); 
		dataArray.data.push_back(transform_right_knee.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_right_knee.getRotation().x()); 
		dataArray.data.push_back(transform_right_knee.getRotation().y()); 
		dataArray.data.push_back(transform_right_knee.getRotation().z()); 
		dataArray.data.push_back(transform_right_knee.getRotation().w());*/

		// right foot joint
		// position
		dataArray.data.push_back(transform_right_foot.getOrigin().x()); 
		dataArray.data.push_back(transform_right_foot.getOrigin().y()); 
		dataArray.data.push_back(transform_right_foot.getOrigin().z());
/*		// orientation
		dataArray.data.push_back(transform_right_foot.getRotation().x()); 
		dataArray.data.push_back(transform_right_foot.getRotation().y()); 
		dataArray.data.push_back(transform_right_foot.getRotation().z()); 
		dataArray.data.push_back(transform_right_foot.getRotation().w());*/		
		
		// published here all the joint position and orientation
		skelData.publish(dataArray);
		
		rate.sleep();
	}
	
	return 0;

};

