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
	#include <opencv2/features2d/features2d.hpp>
    // c++ headers
	#include <iostream>
	// pcl headers
	#include <pcl_ros/point_cloud.h>
	#include <pcl/point_types.h>
	// visualization header
	#include <visualization_msgs/Marker.h>
	#include <pcl/common/time.h> // for watch(timer)
	// Message publication
	#include "std_msgs/Float32MultiArray.h"
	#include <visualization_msgs/Marker.h>

	#include "QuadraticRegression.h"
	#include "LinearRegression.h"
	
	int points =10;

	deque<Point2d> xPositions;
	deque<Point2d> yPositions;
	deque<Point2d> zPositions;
		
	// an instance of quadratic regression
	LinearRegression *xRegression = new LinearRegression();
	QuadraticRegression *yRegression = new QuadraticRegression();
	LinearRegression *zRegression = new LinearRegression();
	

	ros::Publisher pubData;
	
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
	double R[3][3] = { { 0.99984628826577793, 	 0.0012635359098409581,  -0.017487233004436643},
					   {-0.0014779096108364480,  0.99992385683542895,    -0.012251380107679535},
					   { 0.017470421412464927,   0.012275341476520762,    0.99977202419716948 }};
			
	// Translation matrix for depth to RGB conversion 
	double T[3][1] = { {0.019985242312092553}, {-0.00074423738761617583}, {-0.010916736334336222} };
	
	//default capture width and height
	const int FRAME_WIDTH = 640;
	const int FRAME_HEIGHT = 480;
	
	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS=4;
	
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 10*10; // 10pixels X 10pixels
	const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
	
	double x=0,y=0,z=0;
	
	int pixelIndex;
	float dt=0.0;
	
    cv::Mat SE1 = getStructuringElement( CV_SHAPE_ELLIPSE,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat SE2 = getStructuringElement( CV_SHAPE_ELLIPSE,Size(8,8));
	
	// publisher defined
	ros::Publisher marker_pub ;
	
	double dtInstant=0;
	
	// variable declaration
	double xRobot, yRobot, zRobot;

void callback(const ImageConstPtr& image_rgb, const sensor_msgs::PointCloud2::ConstPtr& pCloud)
{
	pcl::StopWatch watch;

	// Solve all of perception here...
	cv::Mat image_color = cv_bridge::toCvCopy(image_rgb)->image;

	// new cloud formation 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	// new cloud from old cloud. (just copy )
	pcl::fromROSMsg (*pCloud, *cloud);
	
	double ball_centre_coord3D[3];	

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
	std_msgs::Float32MultiArray dataArray;
	dataArray.data.clear();
	

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

	imshow("Original", cameraFeed); //show the original image

	// raw depth value from RGB pixel coordinate
	if (u>20 && u<460 && v>20 && v<620 && ballfound)
	{
		pixelIndex = (u) + (v)*pCloud->width;
	}
	else
	{
		ROS_INFO("\n Ball not found in ROI \n");
	}
			
	// 3D coordinates from point cloud using depth value..
	float x3Ddepth = (float)cloud->points[pixelIndex].x;
	float y3Ddepth = (float)cloud->points[pixelIndex].y;
	float z3Ddepth = (float)cloud->points[pixelIndex].z;
	
	// 3D coordinates from depth frame to RGB frame..
	x = (x3Ddepth*R[0][0])+(y3Ddepth*R[0][1])+(z3Ddepth*R[0][2])+T[0][0];
	y = (x3Ddepth*R[1][0])+(y3Ddepth*R[1][1])+(z3Ddepth*R[1][2])+T[1][0];
	z = (x3Ddepth*R[2][0])+(y3Ddepth*R[2][1])+(z3Ddepth*R[2][2])+T[2][0];
	
	ball_centre_coord3D[0]=0; ball_centre_coord3D[1]=0; ball_centre_coord3D[2]=0;
	
	//// Kinect frame to Robot frame..
	//xRobot=z;
	//yRobot=x;
	//zRobot=y;
	
	ball_centre_coord3D[0]=x; ball_centre_coord3D[1]=y; ball_centre_coord3D[2]=z;
	
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

	dtInstant = dtInstant + watch.getTimeSeconds();

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
		
		// XYZ predicted coordinate using least squares method
		if(zPositions[1].x < zPositions[0].x) 
		{
			for(size_t i = 0; i < 30; i++) 
			{
				geometry_msgs::Point p;
				
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
		else 
		{
			for(size_t i = 0; i < 30; i++) 
			{
				geometry_msgs::Point p;
				
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

	// add new point to regression
    xRegression->addPoints(dtInstant, ball_centre_coord3D[0]);
    yRegression->addPoints(dtInstant, ball_centre_coord3D[1]);
	zRegression->addPoints(dtInstant, ball_centre_coord3D[2]);

	cout<<" X=-->>"<<xRobot<<" Y=-->>"<<yRobot<<" Z=-->>"<<zRobot<<endl;
    pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));

	dt=dt+watch.getTimeSeconds();
	
	if(std::isnan(x))
	{
		//ROS_INFO("\n ball not found in ROI \n");
		// data matrix set to zero
		dataArray.data.push_back(0);
		dataArray.data.push_back(0);
		dataArray.data.push_back(0);
		dataArray.data.push_back(dt);
		pubData.publish(dataArray);
	}
	else if (u>20 && u<460 && v>20 && v<620 && ballfound && z !=0)
	{
		// coordinates published at time instant
		dataArray.data.push_back(xRobot);
		dataArray.data.push_back(yRobot);
		dataArray.data.push_back(zRobot);
		dataArray.data.push_back(dt);
		
		calculateTrajectory=true;

		pubData.publish(dataArray);
		
		//printf("\n - Flying ball - %6f %6f %6f %6f", xRobot, yRobot, zRobot, dt);
		pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
	}
	else
	{	
		// coordinates published at time instant
		dataArray.data.push_back(0);
		dataArray.data.push_back(0);
		dataArray.data.push_back(0);
		dataArray.data.push_back(dt);
		
		calculateTrajectory=true;

		pubData.publish(dataArray);
		
		//printf("\n something is wrong \n");
	}

	pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));

    cv::waitKey(1);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	
	// topic subscription
	message_filters::Subscriber<Image> RGB_sub(nh, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud_sub(nh, "/camera/depth/points", 1); //_registered
	// synchronization policy
	typedef sync_policies::ApproximateTime<Image, sensor_msgs::PointCloud2> MySyncPolicy;
	
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), RGB_sub, PointCloud_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

    //pubData = nh.advertise<std_msgs::Float32MultiArray>("pubData", 1);
    // data publish for visualization
	//marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_trajectory", 1);
	
	ros::spin();
	return 0;
}
