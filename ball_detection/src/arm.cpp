#include <ros/ros.h>
// Message publication
#include "std_msgs/Float64.h"
	
	using namespace std;
	
	// publisher declaration
	ros::Publisher shoulder_roll_joint, shoulder_pitch_joint, shoulder_yaw_joint,
						elbow_pitch_joint, elbow_yaw_joint, wrist_pitch_joint, wrist_roll_joint, gripper_joint;



int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_joints");
	ros::NodeHandle nh;

	// publisher declaration

	shoulder_roll_joint = nh.advertise<std_msgs::Float64>("/joint0_controller/command", 1);
	shoulder_pitch_joint = nh.advertise<std_msgs::Float64>("/joint1_controller/command", 1);
	shoulder_yaw_joint = nh.advertise<std_msgs::Float64>("/joint2_controller/command", 1);
	elbow_pitch_joint = nh.advertise<std_msgs::Float64>("/joint3_controller/command", 1);
	elbow_yaw_joint = nh.advertise<std_msgs::Float64>("/joint4_controller/command", 1);
	wrist_pitch_joint = nh.advertise<std_msgs::Float64>("/joint5_controller/command", 1);
	wrist_roll_joint = nh.advertise<std_msgs::Float64>("/joint6_controller/command", 1);
	gripper_joint = nh.advertise<std_msgs::Float64>("/joint7_controller/command", 1);

	
	ros::Rate rate(50.0); // frequency of operation
	
	while (1)
	{	
		std_msgs::Float64 areaValue;
		std_msgs::Float64 Joint0, Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, Joint7;

		Joint0.data = 0.0;
		Joint1.data = -1.2; 
		Joint2.data = 0.0; 
		Joint3.data = -1.2;
		Joint4.data = -1.2; 
		Joint5.data = -1.2;
		Joint6.data = 0.0;
		Joint7.data = 0.0;

		shoulder_roll_joint.publish(Joint0);
		shoulder_pitch_joint.publish(Joint1);
		shoulder_yaw_joint.publish(Joint2);
		elbow_pitch_joint.publish(Joint3);
		elbow_yaw_joint.publish(Joint4);
		wrist_pitch_joint.publish(Joint5);
		wrist_roll_joint.publish(Joint6);
		gripper_joint.publish(Joint7);

	
		rate.sleep();


	}
	return 0;

};

