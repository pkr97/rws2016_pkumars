/***********************************************************************************
Name:           ball13.cpp
Revision:		revision of ball detection and tracking algorithm
Date:           03-03-2016
Author:         Prasanna Kumar Routray
Comments:       for the purpose of ball catching analysis process 
images
Revision:		Revision of ball12.cpp
Libraries:
Notes:          Code generated under Ubuntu 14.04 using ROS indigo, OpenCV, PCL and VTK(for rendering)
Compilation:	catkin_make
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
	#include <sys/time.h>
	#include <chrono>
	#include <geometry_msgs/PoseStamped.h>

	// file headers
	#include "utils.h"
	#include "CGrid_3D.h"

	// c++ headers
	#include <iostream>
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
	
	#include <boost/numeric/ublas/matrix.hpp>
	#include <boost/numeric/ublas/lu.hpp>
	#include <vector>
	#include <stdexcept>
	
	#include <gsl/gsl_vector.h> /*For Vectors*/
    #include <gsl/gsl_rng.h> /* For Random numbers*/
    
    #include "precomp.hpp"
    
    #include <eigen3/Eigen/Eigen>
	#include <opencv2/core/eigen.hpp>
	#include <opencv2/calib3d/calib3d.hpp>
	
	// point cloud definition
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	using namespace sensor_msgs;
	using namespace cv;
	
	// size of the grid for point counting
	double gridsize = BALLRADIUS * 1.6;
	
	// Minimum points inside template
	int min_points = 10;
	
	// height fro roots detection
	double roots_height = 0.40;

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
	
	double roots[3];
	double root_1_coords[3];
	double root_2_coords[3];
	
		// publisher defined
	ros::Publisher marker_pub ;
	ros::Publisher pub;
	ros::Publisher ballCord;
	
	double dt=0.0;
	
	// variable declaration
	double xRobot, yRobot, zRobot;
	
long double getTime()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();
    
	long double milliseconds = td.total_milliseconds();
	
	return milliseconds;
}


void callback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
{
	pcl::StopWatch watch;
	
	min_pt[0]=min_x;
	min_pt[1]=min_y;
	min_pt[2]=min_z;
	
	max_pt[0]=max_x + gridsize;
	max_pt[1]=max_y + gridsize;
	max_pt[2]=max_z + gridsize;
	
	ball_grid_index[0] = 0;
	ball_grid_index[1] = 0;
	ball_grid_index[2] = 0;
	
	root_1_coords[0] = root_1_coords[1] = root_1_coords[2];
	root_2_coords[0] = root_2_coords[1] = root_2_coords[2];
	
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
	
	// Kinect frame to Robot frame..(Transformation)
	xRobot = ball_centre_coord3D[0];
	yRobot = ball_centre_coord3D[1];
	zRobot = ball_centre_coord3D[2];
	double dtInstant = getTime();
	
	geometry_msgs::PoseStamped point_out;
	// KinectEstimated (KE) header stamp and frame id
	point_out.header.stamp = ros::Time::now();
	point_out.header.frame_id = "/ballCord";
	
	if (ball_centre_coord3D[0] !=0)
	{	
		// coordinates published at time instant
	
		// KE Position 
		point_out.pose.position.x = ball_centre_coord3D[0]; 
		point_out.pose.position.y = ball_centre_coord3D[1]; 
		point_out.pose.position.z = ball_centre_coord3D[2];  
		// KE Orientation 
		point_out.pose.orientation.x = dtInstant;  // published in milli seconds
		point_out.pose.orientation.y = 0.0; 
		point_out.pose.orientation.z = 0.0;
		point_out.pose.orientation.w = 1.0;
				
		ballCord.publish(point_out);

		printf("\n%3d - Flying ball - %6f %6f %6f",im_num,ball_centre_coord3D[0],ball_centre_coord3D[1],ball_centre_coord3D[2]);
	}
	else
	{
				
		// KE Position 
		point_out.pose.position.x = 0.0; 
		point_out.pose.position.y = 0.0; 
		point_out.pose.position.z = 0.0;  
		// KE Orientation 
		point_out.pose.orientation.x = dtInstant;  // published in milli seconds
		point_out.pose.orientation.y = 0.0; 
		point_out.pose.orientation.z = 0.0;
		point_out.pose.orientation.w = 1.0;
	
		ballCord.publish(point_out);
	}

	//cout<<"X= "<<ball_centre_coord3D[0]<<" Y= "<<ball_centre_coord3D[1]<<" Z= "<<ball_centre_coord3D[2]<<endl;

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
 
    marker.pose.position.x = ball_centre_coord3D[0];
    marker.pose.position.y = ball_centre_coord3D[1]; 
    marker.pose.position.z = ball_centre_coord3D[2]; 
 
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.10;
    marker.scale.y = 0.10;
    marker.scale.z = 0.10;

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
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, callback);
	
	// data publish for further processing 
	ballCord = nh.advertise<geometry_msgs::PoseStamped>("/ballCord", 1);	
	marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_rviz", 1);	

	ros::spin();
	
	delete(mask);
	
	return 0;
}

