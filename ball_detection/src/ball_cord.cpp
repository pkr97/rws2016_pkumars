/***********************************************************************************
Name:           ball_detect.cpp
Revision:		revision of ball detection and tracking algorithm
Date:           09/05/2016
Author:         Prasanna Kumar Routray
Comments:       for the purpose of ball catching analysis process 
images
Revision:		Revision of ball12.cpp
Libraries:
Notes:          Code generated under Ubuntu 14.04 using ROS indigo, OpenCV, PCL and VTK(for rendering)
Compilation:	catkin_make
Subscription:- 	Point Cloud subscribed in openni_depth_optical_frame to detect flying ball
Publish:-		Ball 3D position published to estimate ball trajectory
***********************************************************************************/	

	//// c++ headers
	#include <iostream>
	// ROS headers
	#include <ros/ros.h>
	#include <sensor_msgs/PointCloud2.h>
	// pcl functions
	#include <pcl/console/print.h> // for printing using pcl
	#include <pcl/common/time.h> // for watch(timer)
	// array manipulation
	#include "std_msgs/Float32MultiArray.h"
	
	// file headers
	#include "utils.h"
	#include "CGrid_3D.h"
	
	// point cloud definition
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	using namespace sensor_msgs;
	
	// size of the grid for point counting
	double gridsize = BALLRADIUS * 1.6;
	
	// Minimum points inside template
	int min_points = 10;
	
	// height fro roots detection
	double roots_height = 0.45;

	int ***mask;

	int im_num = 0;
	int first_run = 1;
	
	Eigen::Vector4f min_pt, max_pt;
	
	CGrid_3D *my_grid;
	
	float min_x = -1.5, min_y = -1.5, min_z = 1.0, max_x = 1.5, max_y = 1.5, max_z = 4.5;
	
	// 3D coordinates of ball
	double ball_centre_coord3D[3];
	double ball_centre_pixel2D[3];
	int ball_grid_index[3];
	
	
	double previous_ball[3];
	
	// publisher defined
	ros::Publisher ballCord;
	
	double dt=0.0;

void callback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
{
	pcl::StopWatch watch;
	
	min_pt[0]=min_x - gridsize;
	min_pt[1]=min_y - gridsize;
	min_pt[2]=min_z - gridsize;
	
	max_pt[0]=max_x + gridsize;
	max_pt[1]=max_y + gridsize;
	max_pt[2]=max_z + gridsize;
	
	ball_grid_index[0] = 0;
	ball_grid_index[1] = 0;
	ball_grid_index[2] = 0;

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

	//////////////////////////////////////////////////////////////////////////////////////////////
	// Ball detection UFO
	ball_centre_coord3D[0]=0;
	ball_centre_coord3D[1]=0;
	ball_centre_coord3D[2]=0;
		
	my_grid->detect_ball(min_points, mask, MASKSIZE, ball_centre_coord3D, 
							ball_grid_index, 0, my_grid->xdim-MASKSIZE, 0, 
								my_grid->ydim-MASKSIZE, 2, my_grid->zdim-MASKSIZE, previous_ball);
	// Ball detection UFO
	//////////////////////////////////////////////////////////////////////////////////////////////
	
	dt=dt+watch.getTimeSeconds(); // running time instant 
	
	geometry_msgs::PointStamped point_out;
	
	if (ball_centre_coord3D[2] !=0)
	{	
		// coordinates published at time instant
		
		// KinectEstimated (KE) header stamp and frame id
		point_out.header.stamp = ros::Time::now();
		point_out.header.frame_id = "/ballCord";
		
		point_out.point.x = ball_centre_coord3D[0];
        point_out.point.y = ball_centre_coord3D[1];
        point_out.point.z = ball_centre_coord3D[2];
	
		ballCord.publish(point_out);

		printf("\n%3d - Flying ball - %6f %6f %6f",im_num,ball_centre_coord3D[0],ball_centre_coord3D[1],ball_centre_coord3D[2]);
	}
	else
	{
		// coordinates published at time instant
		
		// KinectEstimated (KE) header stamp and frame id
		point_out.header.stamp = ros::Time::now();
		point_out.header.frame_id = "/ballCord";
		
		point_out.point.x = 0.0;
        point_out.point.y = 0.0;
        point_out.point.z = 0.0;
	
		ballCord.publish(point_out);
	}
	// frequency print
    pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "detection_node");
	
	ros::NodeHandle nh;	
	
	/////////////////////////////////////////////////////////
	// mask definition for analysis of neighborhood of voxels UFO 
	mask = create_mask(MASKSIZE);
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, callback);
	
	// data publish for further processing 
	ballCord = nh.advertise<geometry_msgs::PointStamped>("/ballCord", 1);	

	ros::spin();
	
	delete(mask);
	
	return 0;
}

