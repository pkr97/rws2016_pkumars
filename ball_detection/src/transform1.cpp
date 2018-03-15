/************************************************************************************
*	Name:           transform.cpp													*
*	Revision:		revision of ball detection and tracking algorithm				*
*	Date:           15-04-2016														*
*	Author:         Prasanna Kumar Routray											*
*	Comments:       for the purpose of getting transformation matrix from point cloud
*																		*
*	Revision:		first version										*
*	Libraries:																		*
*	Notes:          Code generated under Ubuntu 14.04 using ROS indigo, OpenCV, PCL	*
*	Compilation:	catkin_make														*
************************************************************************************/	

	// c++ headers
	#include <iostream>
	// ROS headers
	#include <ros/ros.h>
	// pcl headers
	#include <pcl/common/common.h>
	#include <pcl/common/angles.h>
	#include <pcl/common/transforms.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/io/pcd_io.h>
	#include <pcl/registration/transformation_estimation_svd.h>
	
	using namespace std;

	/**
	 * cloud_Kinect should be from the first frame (From where we are going to take points from Kinect frame to Robot frame)
	 * cloud_Robot should be the second frame (Where we are goint to use the points in robot frame)
	 * the Transform is as follows
	 * 
	 * pointRobot = TKR * pointKinect
	 * 
	 * pointKinect --> point Kinect frame
	 * 
	 * pointRobot --> point Robot frame
	 * 
	 * TKR --> Transformation from Kinect frame to Robot frame
	 * 
	 */
	
int main(int argc, char **argv) 
{
			
	ros::init(argc, argv, "vision_node");
		
	ros::NodeHandle nh;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
	
	cloud_in->width = 6;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->resize(cloud_in->width * cloud_in->height);
	
	cloud_out->width = 6;
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->resize(cloud_out->width * cloud_out->height);
	
	// assume these are kinect points
	cloud_in->points[0].x =  0.1430;
	cloud_in->points[0].y =  1.053;
	cloud_in->points[0].z =  2.59;
	
	cloud_in->points[1].x =  -0.819;
	cloud_in->points[1].y =  0.899;
	cloud_in->points[1].z =  2.728;
	
	cloud_in->points[2].x =  0.789;
	cloud_in->points[2].y =  1.036;
	cloud_in->points[2].z =  3.00;
	
	cloud_in->points[3].x = 0.468;
	cloud_in->points[3].y = 0.894;
	cloud_in->points[3].z = 2.198;
	
	cloud_in->points[4].x = -0.15;
	cloud_in->points[4].y =  0.9070;
	cloud_in->points[4].z = 2.49;
	
	cloud_in->points[5].x = -0.466;
	cloud_in->points[5].y = 0.955;
	cloud_in->points[5].z = 2.87;

/**	
	// make a translation
	// Eigen::Vector3f trans;
	// trans << 0.5,1.0,0.75;


	// assume these are Robot points
	cloud_out->points[0].x = cloud_in->points[0].x + 0.5;
	cloud_out->points[0].y = cloud_in->points[0].y + 1.0;
	cloud_out->points[0].z = cloud_in->points[0].z + 0.75;
	
	cloud_out->points[1].x = cloud_in->points[1].x + 0.5;
	cloud_out->points[1].y = cloud_in->points[1].y + 1.0;
	cloud_out->points[1].z = cloud_in->points[1].z + 0.75;
	
	cloud_out->points[2].x = cloud_in->points[2].x + 0.5;
	cloud_out->points[2].y = cloud_in->points[2].y + 1.0;
	cloud_out->points[2].z = cloud_in->points[2].z + 0.75;
	
	cloud_out->points[3].x = cloud_in->points[3].x + 0.5;
	cloud_out->points[3].y = cloud_in->points[3].y + 1.0;
	cloud_out->points[3].z = cloud_in->points[3].z + 0.75;
	
	cloud_out->points[4].x = cloud_in->points[4].x + 0.5;
	cloud_out->points[4].y = cloud_in->points[4].y + 1.0;
	cloud_out->points[4].z = cloud_in->points[4].z + 0.75;
	
	cloud_out->points[5].x = cloud_in->points[5].x + 0.5;
	cloud_out->points[5].y = cloud_in->points[5].y + 1.0;
	cloud_out->points[5].z = cloud_in->points[5].z + 0.75;
*/
	
	// assume these are Robot points
	cloud_out->points[0].x = 0.2152;
	cloud_out->points[0].y = 1.1755;
	cloud_out->points[0].z = 0.4800165;
	
	cloud_out->points[1].x = 0.7028;
	cloud_out->points[1].y = 0.76185;
	cloud_out->points[1].z = 0.651015;
	
	cloud_out->points[2].x = 0.53625;
	cloud_out->points[2].y = 1.90;
	cloud_out->points[2].z = 0.48539;
	
	cloud_out->points[3].x = 0.71555;
	cloud_out->points[3].y = 1.0165;
	cloud_out->points[3].z = 0.645825;
	
	cloud_out->points[4].x = 0.0186;
	cloud_out->points[4].y = 0.93446;
	cloud_out->points[4].z = 0.649055;
	
	cloud_out->points[5].x = -0.4846;
	cloud_out->points[5].y = 1.095;
	cloud_out->points[5].z = 0.580455;

	// initialization for transform calculation
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformKC; // 4X4 matrix for storing transformation
	
	// Transform calculation
	TESVD.estimateRigidTransformation (*cloud_in,*cloud_out,transformKC);
	
	// Calculated Transform Printing
	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", transformKC (0,0), transformKC (0,1), transformKC (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", transformKC (1,0), transformKC (1,1), transformKC (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", transformKC (2,0), transformKC (2,1), transformKC (2,2));
	printf ("\n");
	printf ("T = < %0.3f, %0.3f, %0.3f >\n", transformKC (0,3), transformKC (1,3), transformKC (2,3));
	
	// new point input
	double xKinect, yKinect, zKinect;
	
	// point to be calculated
	double xRobot, yRobot, zRobot;
	
	// input point initialized
	xKinect = 0.0, yKinect = 0.0, zKinect = 0.0;
	
	// point transform 
	xRobot = (xKinect*transformKC (0,0))+(yKinect*transformKC (0,1))+(zKinect*transformKC (0,2))+transformKC (0,3);
	yRobot = (xKinect*transformKC (1,0))+(yKinect*transformKC (1,1))+(zKinect*transformKC (1,2))+transformKC (1,3);
	zRobot = (xKinect*transformKC (2,0))+(yKinect*transformKC (2,1))+(zKinect*transformKC (2,2))+transformKC (2,3);
	
	// data Printing
	std::cout << "\n The input kinect Coordinate is  : \t" ;
	printf ("pointKinect-----> < %0.3f, %0.3f, %0.3f >\n", xKinect, yKinect, zKinect);
	std::cout << "\n The corresponding Robot Coordinate is  : \t" ;
	printf ("pointRobot-----> < %0.3f, %0.3f, %0.3f >\n", xRobot, yRobot, zRobot);
	
	
	//ros::spin();
	ros::shutdown();
	return 0;
}
	
