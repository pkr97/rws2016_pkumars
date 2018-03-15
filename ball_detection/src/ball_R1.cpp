/***********************************************************************************
Name:           ball_R1.cpp
Revision:		revision of ball detection and tracking algorithm
Date:           03-03-2016
Author:         Prasanna Kumar Routray
Comments:       for the purpose of ball catching analysis process 
images
Revision:		skeleton information added
Libraries:
Notes:          Code generated under Ubuntu 14.04 using ROS indigo, OpenCV, PCL 
Compilation:	catkin_make
***********************************************************************************/	

	// c++ headers
	#include <iostream>
	// ROS synchronization headers
	#include <message_filters/subscriber.h>
	#include <message_filters/synchronizer.h>
	#include <message_filters/sync_policies/approximate_time.h>
	#include <message_filters/time_synchronizer.h>
	// ROS headers
	#include <ros/ros.h>
	#include <sensor_msgs/PointCloud2.h>
	// pcl headers
	#include <pcl/console/print.h>
	#include <pcl/common/time.h> // for watch(timer)
	// Message publication
	#include "std_msgs/Float32MultiArray.h"
	// visualization header
	#include <visualization_msgs/Marker.h>
	// file headers
	#include "utils.h"
	#include "CGrid_3D.h"
	
	// point cloud definition
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	using namespace sensor_msgs;
	using namespace message_filters;
	
	// size of the grid for point counting
	double gridsize = BALLRADIUS * 1.5;
	
	// Minimum points inside template
	int min_points = 10;
	
	// height fro roots detection
	double roots_height = 0.40;

	int ***mask;

	int im_num = 0;
	int first_run = 1;
	
	Eigen::Vector4f min_pt, max_pt;
	
	CGrid_3D *my_grid;
	
	//float min_x = -1.5, min_y = -2.0, min_z = 0.8, max_x = 1.5, max_y = 1.0, max_z = 5.0;
	//float min_x = -1.1, min_y = -1.2, min_z = 0.8, max_x = 1.3, max_y = 1.2, max_z = 4.1;
	float min_x = -1.1, min_y = -1.2, min_z = 0.8, max_x = 1.3, max_y = 1.2, max_z = 4.8;
	
	// 3D coordinates of ball
	double ball_centre_coord3D[3];
	double ball_centre_pixel2D[3];
	int ball_grid_index[3];
	
	
	double previous_ball[3];
	
	// publisher defined
	ros::Publisher marker_pub ;
	ros::Publisher pubData;
	
	double dt=0.0;

void callback(const sensor_msgs::PointCloud2ConstPtr& pCloud, const geometry_msgs::PoseStampedConstPtr& signal)
{
	pcl::StopWatch watch;
	
	cout<<"Hello"<<endl;
	
	min_pt[0]=min_x;
	min_pt[1]=min_y;
	min_pt[2]=min_z;
	
	max_pt[0]=max_x + gridsize;
	max_pt[1]=max_y + gridsize;
	max_pt[2]=max_z + gridsize;
	
	ball_grid_index[0] = 0;
	ball_grid_index[1] = 0;
	ball_grid_index[2] = 0;
	
	// publisher definition
	std_msgs::Float32MultiArray dataArray;
	// data clear 
	dataArray.data.clear();
	
	// new cloud formation 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*pCloud, *cloud);
	

	im_num++;

	if (first_run < 5)
	{
		
		first_run++;
	
		/////////////////////////////////////////
		// Remove points above given distance and below ground
		/////////////////////////////////////////
		for (unsigned int i=0;i<cloud->height;i++)
		{
			for (unsigned int j=0;j<cloud->width;j++)
			{
				unsigned int index_image = i * cloud->width + j;
      
				if (cloud->points[index_image].x > DEPTH_LIMIT)
				{
					cloud->points[index_image].x = sqrt(-1.0); // NaN
					cloud->points[index_image].y = sqrt(-1.0);
					cloud->points[index_image].z = sqrt(-1.0);
				}
			}
		}

		///////////////////////////////////////////////////////////////////////
		// Grid definition (counting number of points in each voxel of the grid)
		if (my_grid)
			my_grid->~CGrid_3D(); // destructor called here 
			my_grid = new CGrid_3D(gridsize,min_pt,max_pt); 
	
	}
	else
	{
		my_grid->reset();
	}
		
	my_grid->compute_grid(cloud);

	//////////////////////////////////////////////////////
	// Ball detection UFO
	ball_centre_coord3D[0]=0;
	ball_centre_coord3D[1]=0;
	ball_centre_coord3D[2]=0;
		
	my_grid->detect_ball(min_points,mask,MASKSIZE, ball_centre_coord3D, ball_grid_index, 0,
									my_grid->xdim-MASKSIZE, 0, my_grid->ydim-MASKSIZE, 2, my_grid->zdim-MASKSIZE, previous_ball);
	// Ball detection UFO
	//////////////////////////////////////////////////////
	
	dt=dt+watch.getTimeSeconds(); // running time instant 
	
	cout<<"Hello"<<endl;
	

	if (ball_centre_coord3D[0] !=0)
	{	
		// coordinates published at time instant
		
		dataArray.data.push_back(ball_centre_coord3D[0]);
		dataArray.data.push_back(ball_centre_coord3D[1]);
		dataArray.data.push_back(ball_centre_coord3D[2]);
		dataArray.data.push_back(dt);
		
		pubData.publish(dataArray);

		printf("\n%3d - Flying ball - %6f %6f %6f",ball_centre_coord3D[0],ball_centre_coord3D[1],ball_centre_coord3D[2]);
		
	}
	else
	{
		dataArray.data.push_back(-signal->pose.position.x);
		dataArray.data.push_back(-signal->pose.position.y);
		dataArray.data.push_back(-signal->pose.position.z);
		dataArray.data.push_back(dt);
		
		pubData.publish(dataArray);
		
		printf("\n%3d - Flying ball - %6f %6f %6f",signal->pose.position.x, -signal->pose.position.y, -signal->pose.position.z);
	}

	// %%%%%%%%%%%%%%%   ball movement visualization in RVIZ  %%%%%%%%%%%%%%%%%%%
	// Visualization marker
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = pCloud->header.frame_id;
    //cout << marker.header.frame_id << endl;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
 
	if (ball_centre_coord3D[0] !=0)
	{
		marker.pose.position.x = ball_centre_coord3D[0];
		marker.pose.position.y = ball_centre_coord3D[1]; 
		marker.pose.position.z = ball_centre_coord3D[2];
	}
	else
	{
		marker.pose.position.x = -signal->pose.position.x;
		marker.pose.position.y = -signal->pose.position.y; 
		marker.pose.position.z = -signal->pose.position.z;
	}
 
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
   
    pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");
	
	ros::NodeHandle nh;	
	/////////////////////////////////////////////////////////
	// mask definition for analysis of neighborhood of voxels UFO 
	mask = create_mask(MASKSIZE);
	
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth/points", 1);
	message_filters::Subscriber<geometry_msgs::PoseStamped> joint_sub(nh, "/left_hand_joint", 1); //_registered
	
	// synchronization policy
	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
	
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, joint_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
	// data publish for further processing 
	pubData = nh.advertise<std_msgs::Float32MultiArray>("/ballCord", 1);	
	marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_rviz", 1);	

	ros::spin();
	
	delete(mask);
	
	return 0;
}

