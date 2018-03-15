/************************************************************************************
*	Name:           ball23.cpp														*
*	Revision:		revision of ball detection and tracking algorithm				*
*	Date:           13-04-2016														*
*	Author:         Prasanna Kumar Routray											*
*	Comments:       for the purpose of ball catching analysis process 				*
*	images																			*
*	Revision:		Revision of ball21.cpp											*
*	Libraries:																		*
*	Notes:          Code generated under Ubuntu 14.04 using ROS indigo, OpenCV, PCL	*
*	Compilation:	catkin_make														*
************************************************************************************/	
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
	#include <geometry_msgs/PoseStamped.h>
	// tf
	#include <tf/transform_listener.h>
	// visualization header
	#include <visualization_msgs/Marker.h>
	// matrix library
	#include </usr/include/armadillo>
	// vtk math library
	#include "vtkMath.h"
	// file headers
	#include "utils.h"
	#include "CGrid_3D.h"
	
	#include "simple_kalman.h"
	#include "QuadraticRegression.h"
	#include "LinearRegression.h"
	
	// point cloud definition
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	using namespace sensor_msgs;

	// past coordinates for regression
	int points =10;

	deque<Point2d> xPositions;
	deque<Point2d> yPositions;
	deque<Point2d> zPositions;
	
	// height for roots detection
	double height = 0.0; // height y-axis of kinect frame (can be changed as per robots suitability)
		
	// an instance of quadratic regression
	LinearRegression *xRegression = new LinearRegression();
	QuadraticRegression *yRegression = new QuadraticRegression();
	LinearRegression *zRegression = new LinearRegression();

	// size of the grid for point counting
	double gridsize = BALLRADIUS * 1.5;
	
	// Minimum points inside template
	int min_points = 10;

	int ***mask;

	int im_num = 0;
	int first_run = 1;

	CGrid_3D *my_grid;

	// publisher defined
	ros::Publisher marker_pub ;
	ros::Publisher kinectEstimeted; // publish end-effector
	ros::Publisher pubData;
	
	double dtInstant=0;
	
	// variable declaration
	double xRobot, yRobot, zRobot;


void callback(const sensor_msgs::PointCloud2ConstPtr& pCloud)
{
	pcl::StopWatch watch;
	
	// listener 
	tf::TransformListener listener;
	
	tf::StampedTransform transform_left_hand;
	
		
	try
	{
		listener.lookupTransform("/left_hand_1", "/openni_depth_optical_frame",ros::Time(0), transform_left_hand);
	}
	catch (tf::TransformException &ex) 
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(0.01).sleep();
	}
	
	// geometry points declaration for storing 3D coordinates of joints and then published later 
	geometry_msgs::Point left_hand_pose;
	
	// left hand joint
	left_hand_pose.x = transform_left_hand.getOrigin().x();
	left_hand_pose.y = transform_left_hand.getOrigin().y();
	left_hand_pose.z = transform_left_hand.getOrigin().z();
	
	Eigen::Vector4f min_pt, max_pt;
	float min_x = -1.5, min_y = -1.8, min_z = 0.6, max_x = 1.5, max_y = 1.8, max_z = 5.0;
	//float min_x = -1.5, min_y = -0.5, min_z = 0.5, max_x = 1.5, max_y = 2.0, max_z = 5.5;
	
	// 3D coordinates of ball
	double ball_centre_coord3D[3];
	int ball_grid_index[3];
	
	
	double previous_ball[3];
	
	double roots[3];
	double root_1_coords[3];
	double root_2_coords[3];
	
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
	ball_centre_coord3D[0]=0; ball_centre_coord3D[1]=0; ball_centre_coord3D[2]=0;
	
			
	my_grid->detect_ball(min_points,mask,MASKSIZE, ball_centre_coord3D, ball_grid_index, 0,
									my_grid->xdim-MASKSIZE, 0, my_grid->ydim-MASKSIZE, 2, my_grid->zdim-MASKSIZE, previous_ball);
	// Ball detection UFO
	//////////////////////////////////////////////////////
	
	if (2>1)
	{
		
	}
	
	xRobot = ball_centre_coord3D[0];
	yRobot = ball_centre_coord3D[1];
	zRobot = ball_centre_coord3D[2];
	
	
	// visualization of trajectory
	visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id = "/camera_depth_optical_frame";
    sphere_list.header.stamp= ros::Time::now();
    sphere_list.ns= "spheres";
    sphere_list.action= visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w= 1.0;
    sphere_list.id = 0;
    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
    
    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.1;
    sphere_list.scale.y = 0.1;
    sphere_list.scale.z = 0.1;

    // Points are green
    sphere_list.color.r = 1.0f;
    sphere_list.color.g = 0.0;
    sphere_list.color.b = 0.0f;
    sphere_list.color.a = 1.0;
	
	bool KalmanPred = true;

	if(zRobot>6.5 || zRobot<0) 
	{
        KalmanPred = false;
	}
	
	//------------MAIN KALMAN CODE-----------//
	
	x=xRobot;
	y=yRobot;
	z=zRobot;

	dtInstant = dtInstant + watch.getTimeSeconds();
	
	double kalman_x = X(0,0);
	double kalman_y = X(2,0);
	double kalman_z = X(4,0);
	
	// Adding the last known position to the deque
    xPositions.push_front(Point2d(dtInstant, ball_centre_coord3D[0]));
    yPositions.push_front(Point2d(dtInstant, ball_centre_coord3D[1]));
    zPositions.push_front(Point2d(dtInstant, ball_centre_coord3D[2]));
    
    if (KalmanPred) 
    {
		
		if(zRegression->getNumberOfPoints() >= 50)
		{
			xRegression->removeOldestPoint();  
			yRegression->removeOldestPoint();
			zRegression->removeOldestPoint();          
		}       
	}
	else 
	{
		xPositions.push_front(Point2f(-1.0f, -1.0f));
		yPositions.push_front(Point2f(-1.0f, -1.0f));
		zPositions.push_front(Point2f(-1.0f, -1.0f));
    }
    
    if(xPositions.size() > points || yPositions.size() > points || zPositions.size() > points) 
    {		
		xPositions.pop_back();
		yPositions.pop_back();
		zPositions.pop_back();
    }
    
    for(size_t i = 1; i < zPositions.size(); i++) 
    {
		if((zPositions[i-1].x < 0 && zPositions[i-1].y < 0) || (zPositions[i].x < 0 && zPositions[i].y < 0)
			|| (xPositions[i-1].x < 0 && xPositions[i-1].y < 0) || (xPositions[i].x < 0 && xPositions[i].y < 0)
			 || (yPositions[i-1].x < 0 && yPositions[i-1].y < 0) || (yPositions[i].x < 0 && yPositions[i].y < 0))
                continue;
    }
    
    if(zRegression->getNumberOfPoints() > 3) 
    {
		double aX = xRegression->getValueA();
		double bX = xRegression->getValueB();
						
		double aY = yRegression->getValueA();
		double bY = yRegression->getValueB();
		double cY = yRegression->getValueC();
		
		double aZ = zRegression->getValueA();
		double bZ = zRegression->getValueB();
		    
		vector<Point2d> zPrediction;
		vector<Point2d> xPrediction;
		vector<Point2d> yPrediction;
		vector<Point3d> trajCoords;
		
		double c1 = aY;
		double c2 = bY;
		double c3 = cY;
		
		double *rootst; // to store returned values
		int num_roots;  // to get no of roots
		/*
		Solves a quadratic equation c1*t^2 + c2*t + c3 = 0 when c1, c2, and c3 are REAL.  Solution is motivated by Numerical Recipes In C 2nd Ed.
		Return array contains number of (real) roots (counting multiple roots as one) followed by roots themselves.
		*/
		rootst = vtkPolynomialSolversUnivariate::SolveQuadratic(c1,c2,c3-height);
  
		roots[0] = rootst[0];
		roots[1] = rootst[1];  // root1 = r1
		roots[2] = rootst[2];  // root2 = r2
		
		cout<<"root1-->="<<roots[1]<<"root2-->="<<roots[2]<<endl;
		
		// root1 and root2 are the time taken to reach the specified plane 
		
		num_roots = rootst[0]; // no of roots
		
		/*
		// 			rootst[3] is for further describing solution
		// roots:	( 0)-no solution; 
		// 			(-1)-infinite number of solutions; 
		// 			( 1)-one distinct real root of multiplicity 2 (stored in r1);
		//			( 2)-two distinct real roots(stored in r1 & r2); 
		//			(-2)-quadratic equation with complex conjugate solution (real part of root returned in r1, imaginary in r2); 
		*/
		
		double timeFlight;
		geometry_msgs::Point p;
		
		if (roots[0]>0)
		{
			if (roots[1]>roots[2])
			{
				timeFlight =  roots[1];
			}
			else
			{
				timeFlight =  roots[2];
			}
		}
		
		if (timeFlight<4)
		{
			/**
			for (double i=0.0; i<timeFlight; i=i+0.033)
			{
				cout<<i<<endl;
				// y-axis
				double x_pred = aX * i + bX;				
                double y_pred = aY * pow(i, 2) + bY * i + cY;
                double z_pred = aZ * i + bZ; 
                
                // point for marker publishing assigned            
                p.x = x_pred;
				p.y = y_pred;
				p.z = z_pred;
				// point for marker published
				sphere_list.points.push_back(p);
            }
            */
            
            double x_pred = aX * timeFlight + bX;				
            double y_pred = aY * pow(timeFlight, 2) + bY * timeFlight + cY;
            double z_pred = aZ * timeFlight + bZ; 
            
            // publisher definition
			geometry_msgs::PoseStamped msg;
		
			// KinectEstimated (KE) header stamp and frame id
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/base1";
		
			//ros::Duration(10).sleep();
		
			// KE Position 
			msg.pose.position.x = x_pred; 
			msg.pose.position.y = y_pred; 
			msg.pose.position.z = z_pred;  
		
			cout<<"x--> "<<msg.pose.position.x<<"  y--> "<<msg.pose.position.y<<"  z--> "<<msg.pose.position.z<<endl;
		
			// KE Orientation 
			msg.pose.orientation.x = 1.0; 
			msg.pose.orientation.y = 0; 
			msg.pose.orientation.z = 0;
			msg.pose.orientation.w = 0;   
	
			// EE Publish 
			kinectEstimeted.publish(msg);
  		}
		else
		{
	
			// XYZ predicted coordinate using least squares fitting method
			if(zPositions[1].x < zPositions[0].x) 
			{
				for(size_t i = 0; i < 30; i++) 
				{
					// x-axis
					double tx = xPositions[0].x + i;
					double x_pred = aX * tx + bX;
					// y-axis				
					double ty = yPositions[0].x + i;
					double y_pred = aY * pow(ty, 2) + bY * ty + cY;
					// z-axis
					double tz = zPositions[0].x + i;
					double z_pred = aZ * tz + bZ; 
					// point for marker publishing assigned            
					p.x = x_pred;
					p.y = y_pred;
					p.z = z_pred;
					// point for marker published
					sphere_list.points.push_back(p);

					xPrediction.push_back(Point2d(tx, x_pred));
					yPrediction.push_back(Point2d(ty, y_pred));
					zPrediction.push_back(Point2d(tz, z_pred));
                
					trajCoords.push_back(Point3d(x_pred, y_pred, z_pred));
				}
			} 
			else 
			{
				for(size_t i = 0; i < 30; i++) 
				{
					double tx = xPositions[0].x + i;
					double x_pred = aX * tx + bX;
								
					double ty = yPositions[0].x + i;
					double y_pred = aY * pow(ty, 2) + bY * ty + cY;
                
					double tz = zPositions[0].x + i;
					double z_pred = aZ * tz + bZ;
				
					p.x = x_pred;
					p.y = y_pred;
					p.z = z_pred;
				
					sphere_list.points.push_back(p);
							
					xPrediction.push_back(Point2d(tx, x_pred));
					yPrediction.push_back(Point2d(ty, y_pred));
					zPrediction.push_back(Point2d(tz, z_pred));
				
					trajCoords.push_back(Point3d(x_pred, y_pred, z_pred));
				}
			}
			sphere_list.lifetime = ros::Duration();
			marker_pub.publish(sphere_list);
		}
	}

	// add new point to regression
    xRegression->addPoints(dtInstant, ball_centre_coord3D[0]);
    yRegression->addPoints(dtInstant, ball_centre_coord3D[1]);
	zRegression->addPoints(dtInstant, ball_centre_coord3D[2]);

	//cout<<" X=-->>"<<xRobot<<" Y=-->>"<<yRobot<<" Z=-->>"<<zRobot<<endl;
    //pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");
	
	ros::NodeHandle nh;
	
	// state initialization
	X 	<< 5.5 << endr
		<< 3.0 << endr
		<< 0.5 << endr
		<< 3.0 << endr
		<< 0.5 << endr
		<< 3.0 << endr;
	// state covariance matrix initialization
	P = 1000 * P;
	// Noise covariance matrix initialization
	Q = 0.01  * Q;
	// Measurement covariance matrix initialization
	R = 0.02  * R;
	
	/////////////////////////////////////////////////////////
	// mask definition for analysis of neighborhood of voxels UFO 
	mask = create_mask(MASKSIZE);
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, callback);
	
	// data publish for visualization
	marker_pub = nh.advertise<visualization_msgs::Marker>("ball_trajectory", 1);
	kinectEstimeted = nh.advertise<geometry_msgs::PoseStamped>("/kinectEstimeted", 1);	

	ros::spin();
	
	delete(mask);
	
	return 0;
}

