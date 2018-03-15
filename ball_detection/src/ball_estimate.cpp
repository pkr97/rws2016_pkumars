/************************************************************************************
*	Name:           ball23.cpp														*
*	Revision:		revision of ball detection and tracking algorithm				*
*	Date:           13-04-2016														*
*	Author:         Prasanna Kumar Routray											*
*	Comments:       for the purpose of ball catching analysis process 				*
*	images																			*
*	Libraries:																		*
*	Notes:          Code generated under Ubuntu 14.04 using ROS indigo and vtk math	*
*	Compilation:	catkin_make														*
*   Subscription:- 	detectes ball coordinate										*
* 	Publish:-		Ball 3D position published to estimate ball trajectory			*
************************************************************************************/
	// c++ headers
	#include <iostream>
	// ROS headers
	#include <ros/ros.h>
	// pcl headers
	#include <pcl/console/print.h>
	#include <pcl/common/time.h> // for watch(timer)
	// Message publication
	#include "std_msgs/Float32MultiArray.h"
	#include <geometry_msgs/PoseStamped.h>
	#include <geometry_msgs/PointStamped.h>
	// vtk math to solve equation
	#include "vtkMath.h"
	// file includes
	#include "simple_kalman.h"
	#include "QuadraticRegression.h"
	#include "LinearRegression.h"
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	
	// to store data for x, y, z positions
	deque<Point2d> xPositions;
	deque<Point2d> yPositions;
	deque<Point2d> zPositions;
	
	// height for roots detection
	double height = 0.40; // height y-axis of kinect frame (can be changed as per robots suitability)
		
	// an instance of quadratic regression
	LinearRegression *xRegression = new LinearRegression();
	QuadraticRegression *yRegression = new QuadraticRegression();
	LinearRegression *zRegression = new LinearRegression();

	// publisher defined
	ros::Publisher kinectEstimeted; // publish end-effector
	
	double dtInstant=0;
	// variable declaration
	double xRobot, yRobot, zRobot;
	// 3D coordinates of ball
	double ball_centre_coord3D[3];
	// to store the roots of quadratic equation
	double roots[3];
	// no of points to be considered for finding coefficients 
	int points = 10;

/**
 * X coordinate is linear fitted
 * Y coordinate is parabolic fitted
 * Z coordinate is linear fitted 
 */


// to estimate the landing point at a certain height with respect to kinect.
void callback(const geometry_msgs::PointStampedConstPtr& msg)
{	
	// clocks starts here
	pcl::StopWatch watch;
	// ball coordinates with time instant from detection_node
	xRobot = msg->point.x;
	yRobot = msg->point.y;
	zRobot = msg->point.z;
	dtInstant = ros::Time::now().toSec();
	
	cout<<dtInstant<<endl;
	
	//geometry_msgs::PointStamped kinect_detections;
	//kinect_detections.header = kinect_dets->header;
	//kinect_detections.header.stamp = ros::Time();
	//kinect_detections.point = kinect_dets->point;
	
	bool prediction = true;

	if(zRobot>4.8 || zRobot<0) 
	{
        prediction = false;
	}
	// Adding the last known position to the deque
    xPositions.push_front(Point2d(dtInstant, xRobot));
    yPositions.push_front(Point2d(dtInstant, yRobot));
    zPositions.push_front(Point2d(dtInstant, zRobot));
    
    if (prediction) 
    {
		if(zRegression->getNumberOfPoints() >= points)
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
		Solves a quadratic equation c1*t^2 + c2*t + c3 = 0 when c1, c2, and c3 are REAL.  
		Solution is motivated by Numerical Recipes In C 2nd Ed.
		Return array contains number of (real) roots (counting multiple roots as one) followed by roots themselves.
		*/
		
		// polynomial solved here to get roots
		rootst = vtkPolynomialSolversUnivariate::SolveQuadratic(c1,c2,c3-height);
  
		roots[0] = rootst[0];
		roots[1] = rootst[1];  // root1 = r1
		roots[2] = rootst[2];  // root2 = r2
		
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
		
		double timeFlight; // time of the flight of the ball
		geometry_msgs::Point p; // landing point
		
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
			cout<<"root1-->="<<roots[1]<<"root2-->="<<roots[2]<<endl;
		}
		
		if (timeFlight<4)
		{
            double x_pred = aX * timeFlight + bX;				
            double y_pred = aY * pow(timeFlight, 2) + bY * timeFlight + cY;
            double z_pred = aZ * timeFlight + bZ; 
            
            // publisher definition
			geometry_msgs::PoseStamped msg;
		
			// KinectEstimated (KE) header stamp and frame id
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/base1";
		
			// KE Position 
			msg.pose.position.x = x_pred; 
			msg.pose.position.y = y_pred; 
			msg.pose.position.z = z_pred;  
			// KE Orientation 
			msg.pose.orientation.x = 1.0; 
			msg.pose.orientation.y = 0.0; 
			msg.pose.orientation.z = 0.0;
			msg.pose.orientation.w = 0.0;   
	
			// EE Publish 
			kinectEstimeted.publish(msg);
			cout<<"x--> "<<msg.pose.position.x<<"  y--> "<<msg.pose.position.y<<"  z--> "<<msg.pose.position.z<<endl;
  		}
		else
		{
			ROS_INFO("Could not estimate in this time");
		}
	}

	// add new point to regression
    xRegression->addPoints(dtInstant, xRobot);
    yRegression->addPoints(dtInstant, yRobot);
	zRegression->addPoints(dtInstant, zRobot);
	
	pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "estimation_node");
	ros::NodeHandle nh;
	
	// subscribe
	ros::Subscriber sub = nh.subscribe ("/ballCord", 1, callback);
	kinectEstimeted = nh.advertise<geometry_msgs::PoseStamped>("/kinectEstimeted", 1);
	
	ros::spin();
	return 0;
}

