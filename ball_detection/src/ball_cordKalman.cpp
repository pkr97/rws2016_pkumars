/***********************************************************************************
Name:           ball17.cpp
Revision:		revision of ball detection and tracking algorithm
Date:           14-03-2016
Author:         Prasanna Kumar Routray
Comments:       for the purpose of ball catching analysis process 
images
Revision:		revision of ball16.cpp (trajectory stuff added)
Libraries:
Notes:          Code generated under Ubuntu using ROS, OpenCV, PCL
Compilation:	catkin_make
***********************************************************************************/

	// ROS headers
	#include <ros/ros.h>
    // c++ headers
	#include <iostream>
	// pcl headers
	#include <pcl/console/print.h>
	#include <pcl/common/time.h> // for watch(timer)
	// Message publication
	#include "std_msgs/Float32MultiArray.h"
	#include <geometry_msgs/PoseStamped.h>
	// matrix library
	#include </usr/include/armadillo>
	// vtk math library
	#include "vtkMath.h"
	
	#include "simple_kalman.h"
	#include "QuadraticRegression.h"
	#include "LinearRegression.h"

	ros::Publisher pubSensorData;
		
	int points = 15;
	
	// height for roots detection
	double height = 0.0; // height y-axis of kinect frame (can be changed as per robots suitability)

	deque<Point2d> xPositions;
	deque<Point2d> yPositions;
	deque<Point2d> zPositions;
		
	// an instance of quadratic regression
	LinearRegression *xRegression = new LinearRegression();
	QuadraticRegression *yRegression = new QuadraticRegression();
	LinearRegression *zRegression = new LinearRegression();
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;

		// Kinect data matrix
	double ballCord[20][04] = { { 3.946666955947876, -0.17232169210910797, -0.3031330108642578, 342.4720153808594},
								{ 3.860928773880005, -0.16258779168128967, -0.41441142559051514, 342.50701904296875},
								{ 3.598681926727295, -0.16141949594020844, -0.5733663439750671, 342.5370178222656 },
								{ 3.528154134750366, -0.15319129824638367, -0.6354023814201355, 342.5670166015625},
								{ 3.400434970855713, -0.14658403396606445, -0.6966089606285095, 342.5980224609375},
								{ 3.2748520374298096, -0.13998772203922272, -0.7477661371231079, 342.6300048828125},
								{ 3.1735641956329346, -0.12534910440444946, -0.7814740538597107, 342.6610107421875},
								{ 3.081439256668091, -0.12036224454641342, -0.8038467764854431, 342.6910095214844},
								{ 2.842771530151367, -0.11406166106462479, -0.8201966881752014, 342.72100830078125},
								{ 2.7250001430511475, -0.09760880470275879, -0.8154234886169434, 342.7510070800781},
								{ 2.6009461879730225, -0.08685405552387238, -0.7948941588401794, 342.781005859375},
								{ 2.4926459789276123, -0.07774360477924347, -0.768384575843811, 342.8130187988281},
								{ 2.371255397796631, -0.07264704257249832, -0.7300833463668823, 342.8450012207031},
								{ 2.1279220581054688, -0.054890576750040054, -0.6023528575897217, 342.8760070800781},
								{ 2.004295587539673, -0.04633328318595886, -0.5297168493270874, 342.9070129394531},
								{ 1.886982798576355, -0.04055865481495857, -0.4516644775867462, 342.93902587890625},
								{ 1.7609566450119019, -0.034193456172943115, -0.3481166958808899, 342.97100830078125},
								{ 1.638879656791687, -0.023798108100891113, -0.23560139536857605, 343.0090026855469},
								{ 1.388088583946228, -0.00603143498301506, 0.021812263876199722, 343.03900146484375},
								{ 1.079174280166626, 0.019643785431981087, 0.42968496680259705, 343.1330261230469}};	
								
double dtInstant=0;

ros::Publisher robotEstimeted; // publish end-effector

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	
	while (ros::ok())
	{	
	double xOffset, yOffset, zOffset;
	
	// in meters
	xOffset =  0.00;
	yOffset = -0.06; 
	zOffset = -0.05;
	
	// state initialization
	X 	<< 7.5 << endr
		<< 3.0 << endr
		<< 0.5 << endr
		<< 3.0 << endr
		<<-0.5 << endr
		<< 3.0 << endr;
	// state covariance matrix initialization
	P = 1000 * P;
	// Noise covariance matrix initialization
	Q = 0.1  * Q;
	// Measurement covariance matrix initialization
	R = 0.2  * R;

	
	robotEstimeted = nh.advertise<geometry_msgs::PoseStamped>("/robotEstimeted", 1);
	
	
	
	for(size_t count = 0; count < 20; count++) 
	{
		pcl::StopWatch watch;
		
		double xRobot, yRobot, zRobot;
		// Kinect frame to Robot frame..
		xRobot = ballCord[count][1];
		yRobot = ballCord[count][2];
		zRobot = -ballCord[count][3];
			
		bool KalmanPred = true;

		if(x>6.5 || x<0) 
		{	
			KalmanPred = false;
		}
		
		//------------MAIN KALMAN CODE-----------//
		
		x=xRobot;
		y=yRobot;
		z=zRobot;
		dtInstant = ballCord[count][4] - ballCord[1][4];
		
		double kalman_x = X(0,0);
		double kalman_y = X(2,0);
		double kalman_z = X(4,0);
	
		// Adding the last known position to the deque
		xPositions.push_front(Point2d(dtInstant, xRobot));
		yPositions.push_front(Point2d(dtInstant, yRobot));
		zPositions.push_front(Point2d(dtInstant, zRobot));
		
		// Calculate velocity from coordinates.
		if (KalmanPred) 
		{
			v_x = (x - old_x)/dt;
			v_y = (y - old_y)/dt;
			v_z = (z - old_z)/dt;
			
			dt = ballCord[count+1][4] - ballCord[count][4];

			if((abs(v_x>30) || abs(v_y)>30 || abs(v_z)>30 || abs(old_y - 999.999) < 0.01) && !Kalman_flag) 
			{
				old_x = x;
				old_y = y;
				old_z = z;
				KalmanPred = false;
			}
		
		}
		
		if (KalmanPred) 
		{
			old_x = x;
			old_y = y;
			old_z = z;

			if(!v_x && !v_y && !v_z) 
			{
				KalmanPred = false;
			}
		}
	
		if (KalmanPred) 
		{
			Kalman_flag = true;
			Kalman_calc(x , v_x, y , v_y, z , v_z, dt);
			        
			// update 
			double kalman_x = X(0,0);
			double kalman_y = X(2,0);
			double kalman_z = X(4,0);
	
			if(zRegression->getNumberOfPoints() >= 15)
			{
				zRegression->removeOldestPoint();   
				xRegression->removeOldestPoint();  
				yRegression->removeOldestPoint();       
			}
        
		}
	
		else 
		{
			zPositions.push_front(Point2f(-1.0f, -1.0f));
			xPositions.push_front(Point2f(-1.0f, -1.0f));
			yPositions.push_front(Point2f(-1.0f, -1.0f));
		}
		
		if(zPositions.size() > points || xPositions.size() > points || yPositions.size() > points) 
		{
			zPositions.pop_back();
			xPositions.pop_back();
			yPositions.pop_back();
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
			
			double roots[3];
		
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
			
			if (timeFlight<3)
			{
				for (double i=0.0; i<timeFlight; i=i+0.033)
				{
					cout<<i<<endl;
					// y-axis
					double x_pred = aX * i + bX;				
					double y_pred = aY * pow(i, 2) + bY * i + cY;
					double z_pred = aZ * i + bZ;
					
					// publisher definition
					geometry_msgs::PoseStamped robotEE;
					
					// EE header stamp and frame id
					robotEE.header.stamp = ros::Time::now();
					robotEE.header.frame_id = "/base1";
					
					// EE Position 
					robotEE.pose.position.x =  x_pred + xOffset; 
					robotEE.pose.position.y = -y_pred + xOffset; 
					robotEE.pose.position.z =  z_pred + xOffset;
					
					// EE Orientation 
					robotEE.pose.orientation.x = 1.0; 
					robotEE.pose.orientation.y = 0; 
					robotEE.pose.orientation.z = 0;
					robotEE.pose.orientation.w = 0;
					
					// EE Publish 
					robotEstimeted.publish(robotEE);
                }
            
			}
			else
			{
			
				// XYZ predicted coordinate using least squares method
				if(zPositions[1].x < zPositions[0].x) 
				{
					for(size_t i = 0; i < 30; i++) 
					{
						
						double tx = xPositions[0].x + i;
		                double x_pred = aX * tx + bX;
										
		                double ty = yPositions[0].x + i;
		                double y_pred = aY * pow(ty, 2) + bY * ty + cY;
		                
		                double tz = zPositions[0].x + i;
		                double z_pred = aZ * tz + bZ;
		               
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
						
														
						xPrediction.push_back(Point2d(tx, x_pred));
						yPrediction.push_back(Point2d(ty, y_pred));
						zPrediction.push_back(Point2d(tz, z_pred));
						
						trajCoords.push_back(Point3d(x_pred, y_pred, z_pred));
					}
				}
			}
			
		}
	
		// add new point to regression
	    xRegression->addPoints(dtInstant, xRobot);
	    yRegression->addPoints(dtInstant, yRobot);
		zRegression->addPoints(dtInstant, zRobot);
		
		pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
	}
    
    
    
}
	return 0;
}
