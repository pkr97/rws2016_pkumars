#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_skeleton_tf_listener");
	
	ros::NodeHandle node;

	// publisher declaration
	ros::Publisher left_hand_joint = node.advertise<geometry_msgs::PoseStamped>("/left_hand_joint", 1);
		
	// listener 
	tf::TransformListener listener;
	
	ros::Rate rate(30.0); // frequency of operation
	
	while (node.ok())
	{
		// Transforms declared for each joint
		tf::StampedTransform transform_left_hand;
		
		try
		{
			// each joint frame to reference frame transforms
			listener.lookupTransform("/left_hand_1", "/openni_depth_frame",ros::Time(0), transform_left_hand);
		}
			catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(0.0001).sleep();
			continue;
		}
	
		// geometry points declaration for storing 3D coordinates of joints and then published later 
		geometry_msgs::PoseStamped left_hand_pose;
		
		left_hand_pose.header.stamp = ros::Time::now();
		
		left_hand_pose.header.frame_id = "/pose6";
		
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


		// joint positions publish
		left_hand_joint.publish(left_hand_pose);
		
		
		rate.sleep();
	}
	return 0;
};

