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
	
	
	// ROS synchronization headers
	#include <message_filters/subscriber.h>
	#include <message_filters/synchronizer.h>
	#include <message_filters/sync_policies/approximate_time.h>
	#include <message_filters/time_synchronizer.h>
	// ROS headers
	#include <ros/ros.h>
	#include <sensor_msgs/Image.h>
	#include <sensor_msgs/CameraInfo.h>
	#include <sensor_msgs/PointCloud2.h>
	#include <image_transport/image_transport.h>
	#include <sensor_msgs/image_encodings.h>
	// OpenCV headers
	#include <cv_bridge/cv_bridge.h>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/core/core.hpp>
	#include <opencv2/opencv.hpp>
	#include <opencv2/features2d/features2d.hpp>
    // c++ headers
	#include <iostream>
	// pcl headers
	#include <pcl_ros/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/console/print.h>
	#include <pcl/common/time.h> // for watch(timer)
	// visualization header
	#include <visualization_msgs/Marker.h>
	// Message publication
	#include "std_msgs/Float32MultiArray.h"
	#include <visualization_msgs/Marker.h>
	// matrix library
	#include </usr/include/armadillo>
	
	#include "simple_kalman.h"
	#include "QuadraticRegression.h"
	#include "LinearRegression.h"

	ros::Publisher pubSensorData;
	ros::Publisher pubKalmanData;
		
	int points =30;

	deque<Point2d> xPositions;
	deque<Point2d> yPositions;
	deque<Point2d> zPositions;
	
	// an instance of quadratic regression
	QuadraticRegression *zRegression = new QuadraticRegression();
	
	LinearRegression *xRegression = new LinearRegression();
	
	LinearRegression *yRegression = new LinearRegression();

	
	// point cloud definition
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	using namespace sensor_msgs;
	using namespace message_filters;
	using namespace cv;

	// Blue Ball.
	int H_MIN = 61;
	int H_MAX = 160;
	int S_MIN = 125;
	int S_MAX = 256;
	int V_MIN = 100;
	int V_MAX = 256;

	// Rotation matrix for depth to RGB conversion 
	double Rotation[3][3] = { { 0.99984628826577793, 	 0.0012635359098409581,  -0.017487233004436643},
					   {-0.0014779096108364480,  0.99992385683542895,    -0.012251380107679535},
					   { 0.017470421412464927,   0.012275341476520762,    0.99977202419716948 }};
			
	// Translation matrix for depth to RGB conversion 
	double Translation[3][1] = { {0.019985242312092553}, {-0.00074423738761617583}, {-0.010916736334336222} };
	
	//default capture width and height
	const int FRAME_WIDTH = 640;
	const int FRAME_HEIGHT = 480;
	
	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS=4;
	
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 10*10; // 10pixels X 10pixels
	const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
	
	int pixelIndex;
	double dtInstant=0;
	
    cv::Mat SE1 = getStructuringElement( CV_SHAPE_ELLIPSE,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat SE2 = getStructuringElement( CV_SHAPE_ELLIPSE,Size(8,8));



void callback(const ImageConstPtr& image_rgb, const sensor_msgs::PointCloud2::ConstPtr& pCloud)
{
	pcl::StopWatch watch;
	// Solve all of perception here...
	cv::Mat image_color = cv_bridge::toCvCopy(image_rgb)->image;

	// new cloud formation 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	// new cloud from old cloud. (just copy )
	pcl::fromROSMsg (*pCloud, *cloud);
	
	// new image frame. to get size of point cloud
	cv::Mat imageFrame;
	imageFrame = cv::Mat(cloud->height, cloud->width, CV_8UC3); 
	
	for (int h=0; h<imageFrame.rows; h++) 
    {
        for (int w=0; w<imageFrame.cols; w++) 
        {
            pcl::PointXYZ point = cloud->at(w, h);
        }
    }

	// publisher definition
	std_msgs::Float32MultiArray sensorDataArray;
	sensorDataArray.data.clear();
	
	std_msgs::Float32MultiArray kalmanDataArray;
	kalmanDataArray.data.clear();
	

	cv::Mat cameraFeed;
	//matrix storage for HSV image
	cv::Mat HSV;
	//matrix storage for binary threshold image
	cv::Mat threshold;

	//x and y values for the location of the object
	int u = 0, v = 0;
	image_color.copyTo(cameraFeed);
	
	//convert frame from BGR to HSV colorspace
	cv::cvtColor(cameraFeed,HSV,CV_BGR2HSV);
	
	//filter HSV image between values and store filtered image to threshold matrix
	cv::inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);

	cv::erode(threshold,threshold,SE1);
	cv::erode(threshold,threshold,SE1);
		
	cv::dilate(threshold,threshold,SE2);
	cv::dilate(threshold,threshold,SE2);

	dilate(threshold,threshold,SE2);

	cv::Mat temp;
	threshold.copyTo(temp);
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	
	double refArea = 0;
	bool calculateTrajectory=false;
	bool ballfound = false;
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS)
        {
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 10 px by 10px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                {
					u = moment.m10/area;
					v = moment.m01/area;
					ballfound = true;
					refArea = area;
				}
				else
				{
					ballfound = false;
				}
			}
		}
	}

	imshow("Thresholded Image", threshold); //show the thresholded image

	// raw depth value from RGB pixel coordinate
	if (u>20 && u<460 && v>20 && v<620 && ballfound)
	{
		pixelIndex = (u) + (v)*cloud->width;
	}
	else
	{
		ROS_INFO("\n Ball not found in ROI \n");
	}
			
	// 3D coordinates from point cloud using depth value..
	double x3Ddepth = (double)cloud->points[pixelIndex].x;
	double y3Ddepth = (double)cloud->points[pixelIndex].y;
	double z3Ddepth = (double)cloud->points[pixelIndex].z;
	
	// 3D coordinates from depth frame to RGB frame..
	double Kinect_x = (x3Ddepth*Rotation[0][0])+(y3Ddepth*Rotation[0][1])+(z3Ddepth*Rotation[0][2])+Translation[0][0];
	double Kinect_y = (x3Ddepth*Rotation[1][0])+(y3Ddepth*Rotation[1][1])+(z3Ddepth*Rotation[1][2])+Translation[1][0];
	double Kinect_z = (x3Ddepth*Rotation[2][0])+(y3Ddepth*Rotation[2][1])+(z3Ddepth*Rotation[2][2])+Translation[2][0];
	
	double xRobot, yRobot, zRobot;
	// Kinect frame to Robot frame..
	xRobot=Kinect_z;
	yRobot=Kinect_x;
	zRobot=Kinect_y;

	

	if(std::isnan(xRobot))
	{
		ROS_INFO("\n ball not found in ROI \n");
		// data matrix set to zero
		sensorDataArray.data.push_back(0);
		sensorDataArray.data.push_back(0);
		sensorDataArray.data.push_back(0);
		sensorDataArray.data.push_back(0);
	}
	else if (u>20 && u<460 && v>20 && v<620 && ballfound && xRobot !=0)
	{
		calculateTrajectory=true;

		printf("\n - Flying ball - %6f %6f %6f ", xRobot, yRobot, zRobot ); //dt
		//pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
	}
	else
	{
		printf("\n something is wrong \n");
	}

	x=xRobot;
	y=yRobot;
	z=zRobot;
	
	
	bool KalmanPred = true;

	if(x>6.5 || x<0) 
	{	
		if(z>4)
		{
			circle(cameraFeed, Point(u, v), 20, Scalar(255, 0, 0), 5);
        }
        else if(z<0)
        {
			circle(cameraFeed, Point(u, v), 20, Scalar(0, 255, 0), 5);
        }
        else
        {
			circle(cameraFeed, Point(u, v), 20, Scalar(255,255,0), 5);
		}
        KalmanPred = false;
	}

	//------------MAIN KALMAN CODE-----------//

	// Calculate velocity from coordinates.
	if (KalmanPred && calculateTrajectory) 
	{
		v_x = (x - old_x)/dt;
		v_y = (y - old_y)/dt;
		v_z = (z - old_z)/dt;

		dt = watch.getTimeSeconds();			

		if((abs(v_x>30) || abs(v_y)>30 || abs(v_z)>30 || abs(old_y - 999.999) < 0.01) && !Kalman_flag) 
		{
			old_x = x;
			old_y = y;
			old_z = z;
			KalmanPred = false;
		}
		
	}
	
	if (KalmanPred && calculateTrajectory) 
    {
		old_x = x;
        old_y = y;
        old_z = z;

        if(!v_x && !v_y && !v_z) 
        {
			KalmanPred = false;
		}
	}
	
	dtInstant = dtInstant + watch.getTimeSeconds();
	
	double kalman_x = X(0,0);
	double kalman_y = X(2,0);
	double kalman_z = X(4,0);
	
	// Adding the last known position to the deque
    zPositions.push_front(Point2d(dtInstant, kalman_z));
    xPositions.push_front(Point2d(dtInstant, kalman_x));
    yPositions.push_front(Point2d(dtInstant, kalman_y));
	
	
	if (KalmanPred && calculateTrajectory) 
    {
		Kalman_flag = true;
        Kalman_calc(x , v_x, y , v_y, z , v_z, dt);
        circle(cameraFeed, Point(u,v), 20, Scalar(0, 0, 255), 5);
        
        // update 
        double kalman_x = X(0,0);
		double kalman_y = X(2,0);
		double kalman_z = X(4,0);
	    
	    // coordinates from sensor published at time instant
		sensorDataArray.data.push_back(xRobot);
		sensorDataArray.data.push_back(yRobot);
		sensorDataArray.data.push_back(zRobot);
		sensorDataArray.data.push_back(dtInstant);
		
		sensorDataArray.data.push_back(kalman_x);
		sensorDataArray.data.push_back(kalman_y);
		sensorDataArray.data.push_back(kalman_z);
		sensorDataArray.data.push_back(dtInstant);
        
        /**
        // coordinates from Kalman published at time instant
		kalmanDataArray.data.push_back(kalman_x);
		kalmanDataArray.data.push_back(kalman_y);
		kalmanDataArray.data.push_back(kalman_z);
		kalmanDataArray.data.push_back(dtInstant);
		*/
		// data published here 
		pubSensorData.publish(sensorDataArray);
		//pubKalmanData.publish(sensorDataArray);
		
		if(zRegression->getNumberOfPoints() >= 50)
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
                

        //int lineThickness = sqrt(points / (i+1)) * 2.5;
        //line(bgrImage, zPositions[i-1], zPositions[i], Scalar(0, 0, 255), lineThickness);
    }
    
    if(zRegression->getNumberOfPoints() > 3) 
    {
		//cout << "Y = " << realRegression->getValueA() << "X2 + " << realRegression->getValueB() << "X + " << realRegression->getValueC() << endl;
    
		double aZ = zRegression->getValueA();
		double bZ = zRegression->getValueB();
		double cZ = zRegression->getValueC();
		
		double aX = xRegression->getValueA();
		double bX = xRegression->getValueB();
				
		double aY = yRegression->getValueA();
		double bY = yRegression->getValueB();
    
		vector<Point> zPrediction;
		vector<Point> xPrediction;
		vector<Point> yPrediction;
		
		// Z coordinate 
		if(zPositions[1].x < zPositions[0].x) 
		{
			for(size_t i = 0; i < 100; i++) 
			{
				double t = zPositions[0].x + i;
                double z_pred = aZ * pow(t, 2) + bZ * t + cZ;
                zPrediction.push_back(Point2d(t, z_pred));
            }
                
                
		} 
		else 
		{
			for(size_t i = 0; i < 100; i++) 
			{
				double t = zPositions[0].x - i;
				double z_pred = aZ * pow(t, 2) + bZ * t + cZ;
				zPrediction.push_back(Point(t, z_pred));
			}
		}
		// X coordinate 
		if(xPositions[1].x < xPositions[0].x) 
		{
			for(size_t i = 0; i < 100; i++) 
			{
				double t = xPositions[0].x + i;
                double x_pred = aX * t + bX;
                xPrediction.push_back(Point2d(t, x_pred));
            }
                
                
		} 
		else 
		{
			for(size_t i = 0; i < 100; i++) 
			{
				double t = xPositions[0].x - i;
				double x_pred = aX * t + bX;
				xPrediction.push_back(Point(t, x_pred));
			}
		}
		// y coordinate 
		if(yPositions[1].x < yPositions[0].x) 
		{
			for(size_t i = 0; i < 100; i++) 
			{
				double t = yPositions[0].x + i;
                double y_pred = aY * t + bY;
                yPrediction.push_back(Point2d(t, y_pred));
            }
                
                
		} 
		else 
		{
			for(size_t i = 0; i < 100; i++) 
			{
				double t = yPositions[0].x - i;
				double y_pred = aY * t + bY;
				yPrediction.push_back(Point(t, y_pred));
			}
		}
    
		for(size_t i = 0; i < zPrediction.size()-1; i++) 
		{
			line(cameraFeed, zPrediction[i], zPrediction[i+1], Scalar(255, 0, 0), 2.5);
		}
		
		for(size_t i = 0; i < xPrediction.size()-1; i++) 
		{
			line(cameraFeed, xPrediction[i], xPrediction[i+1], Scalar(0, 255, 0), 2.5);
		}
		
		for(size_t i = 0; i < yPrediction.size()-1; i++) 
		{
			line(cameraFeed, yPrediction[i], yPrediction[i+1], Scalar(0, 255, 0), 2.5);
		}
	}

    // add new point
    zRegression->addPoints(dtInstant, kalman_z);
    xRegression->addPoints(dtInstant, kalman_x);
    yRegression->addPoints(dtInstant, kalman_y);
    
	
	
    imshow("Original", cameraFeed); //show the original image     
	
	cv::waitKey(1);
	
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	
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
	
	// topic subscription
	message_filters::Subscriber<Image> RGB_sub(nh, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud_sub(nh, "/camera/depth/points", 1);
	
	// synchronization policy
	typedef sync_policies::ApproximateTime<Image, sensor_msgs::PointCloud2> MySyncPolicy;
	
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), RGB_sub, PointCloud_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
	// data publisher
    pubSensorData = nh.advertise<std_msgs::Float32MultiArray>("pubSensorData", 10);
    //pubKalmanData = nh.advertise<std_msgs::Float32MultiArray>("pubKalmanData", 10);
	
	ros::spin();
	return 0;
}
