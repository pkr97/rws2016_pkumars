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
	// pcl functions
	#include <pcl/common/time.h> // for watch(timer)
	#include <pcl/console/print.h>
	// visualization header
	#include <visualization_msgs/Marker.h>
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
	
	// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
	using namespace std;
	using namespace sensor_msgs;
	using namespace message_filters;
	using namespace cv;

// ############################# Structuring Elements 	START		############################
    cv::Mat SE1 = getStructuringElement( CV_SHAPE_ELLIPSE,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat SE2 = getStructuringElement( CV_SHAPE_ELLIPSE,Size(8,8));
// ############################# Structuring Elements 	END			############################
	
// ############################# KINECT Camera Parameters START		############################
	// Color
	double fx_rgb = 5.2921508098293293e+02;
	double fy_rgb = 5.2556393630057437e+02;
	double cx_rgb = 3.2894272028759258e+02;
	double cy_rgb = 2.6748068171871557e+02;
	double k1_rgb = 2.6451622333009589e-01;
	double k2_rgb = -8.3990749424620825e-01;
	double p1_rgb = -1.9922302173693159e-03;
	double p2_rgb = 1.4371995932897616e-03;
	double k3_rgb = 9.1192465078713847e-01;
	
	// Depth
	double fx_d = 5.9421434211923247e+02;
	double fy_d = 5.9104053696870778e+02;
	double cx_d = 3.3930780975300314e+02;
	double cy_d = 2.4273913761751615e+02;
	double k1_d = -2.6386489753128833e-01;
	double k2_d = 9.9966832163729757e-01;
	double p1_d = -7.6275862143610667e-04;
	double p2_d = 5.0350940090814270e-03;
	double k3_d = -1.3053628089976321e+00;
	
	// Rotation matrix for depth to RGB conversion 
	double R[3][3] = { { 0.99984628826577793, 	 0.0012635359098409581,  -0.017487233004436643},
					   {-0.0014779096108364480,  0.99992385683542895,    -0.012251380107679535},
					   { 0.017470421412464927,   0.012275341476520762,    0.99977202419716948 }};
			
	// Translation matrix for depth to RGB conversion 
	double T[3][1] = { {0.019985242312092553}, {-0.00074423738761617583}, {-0.010916736334336222} };
	
	//default capture width and height
	const int FRAME_WIDTH = 640;
	const int FRAME_HEIGHT = 480;
// ############################# KINECT Camera Parameters END		#############################

// ########################## Ball Extraction parameters START		###########################
	// Blue Ball.
	int H_MIN = 61;
	int H_MAX = 160;
	int S_MIN = 125;
	int S_MAX = 256;
	int V_MIN = 100;
	int V_MAX = 256;


	
	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS=4;
	
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 10*10; // 10pixels X 10pixels
	const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
	
	double x=0,y=0,z=0;
	
	int pixelIndex;
	float dt=0.0;
	
	// publisher defined
	ros::Publisher marker_pub ;
	
	double dtInstant=0;
	
	// variable declaration
	double xRobot, yRobot, zRobot;
// ########################## Ball Extraction parameters END		############################

// ###################	Image Processing and 3D position Extraction callback function		#####

void callback(const ImageConstPtr& image_rgb, const ImageConstPtr& image_depth_source)
{
	pcl::StopWatch watch;

	// Solve all of perception here... (ROS Format converted to OpenCV format here)
	cv::Mat image_color = cv_bridge::toCvCopy(image_rgb)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_source)->image;

	// publisher definition
	std_msgs::Float32MultiArray dataArray;
	// data clear 
	dataArray.data.clear();
	double ball_centre_coord3D[3];
	
	// ###################        Image processing starts here		############################
	
	// new matrix declaration for copy and processing 
	cv::Mat cameraFeed;
	//matrix storage for HSV image
	cv::Mat HSV;
	//matrix storage for binary threshold image
	cv::Mat threshold;

	//x and y values for the location of the object
	int u = 0, v = 0;
	double x=0,y=0,z=0;
	image_color.copyTo(cameraFeed);
	
	//convert frame from BGR to HSV colorspace
	cv::cvtColor(cameraFeed,HSV,CV_BGR2HSV);
	
	//filter HSV image between values and store filtered image to threshold matrix
	cv::inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
	// erosion 
	cv::erode(threshold,threshold,SE1);
	cv::erode(threshold,threshold,SE1);
	// dilation
	cv::dilate(threshold,threshold,SE2);
	cv::dilate(threshold,threshold,SE2);
	// dilation 
	dilate(threshold,threshold,SE2);
	// ###################        Image processing ends here		############################
	
	// ###################        Ball detection process          ##############################
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
				// moment calculation 
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 10 px by 10px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                {	
					// pixel coordinate of ball
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
	// ############ 	Ball detection process ends here with pixel coordinates		#################
	
	// ############ 	Ball 3D position extraction process starts here 		#####################
	
	// Coordinates defined
	double x3Ddepth, y3Ddepth, z3Ddepth;
	
	if (u>20 && u<460 && v>20 && v<620 && ballfound)
	{
		// 3D coordinates from point cloud using depth value.. in Kinect coordinate space
		z3Ddepth = image_depth.at<ushort>(Point2d(u,v)) / 1000.0f;
		x3Ddepth = (u - cx_d) * z3Ddepth / fx_d;
		y3Ddepth = (v - cy_d) * z3Ddepth / fy_d;

		// 3D coordinates from depth frame to RGB frame.. (Transformed using Rotation and Translation matrix)
		x = (x3Ddepth*R[0][0])+(y3Ddepth*R[0][1])+(z3Ddepth*R[0][2])+T[0][0];
		y = (x3Ddepth*R[1][0])+(y3Ddepth*R[1][1])+(z3Ddepth*R[1][2])+T[1][0];
		z = (x3Ddepth*R[2][0])+(y3Ddepth*R[2][1])+(z3Ddepth*R[2][2])+T[2][0];
	}
	
	if (std::isnan(x))
	{
		x=0.0;
		y=0.0;
		z=0.0;
	}
	else if(z<0)
	{
		x=0.0;
		y=0.0;
		z=0.0;
	}
/**
	
	xRobot = x;
	yRobot = y;
	zRobot = z;
	
	
	
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
    xPositions.push_front(Point2d(dtInstant, x));
    yPositions.push_front(Point2d(dtInstant, y));
    zPositions.push_front(Point2d(dtInstant, z));
    
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
    xRegression->addPoints(dtInstant, x);
    yRegression->addPoints(dtInstant, y);
	zRegression->addPoints(dtInstant, z);
*/
	cout<<" X=-->>"<<xRobot<<" Y=-->>"<<yRobot<<" Z=-->>"<<zRobot<<endl;
    pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));

    cv::waitKey(1);	
    
    // ############			Image Dispaly and Refresh ends here		################################
}

// ################			main function START		################################################ 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	
	// topic subscription
	message_filters::Subscriber<Image> RGB_sub(nh, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<Image> DEPTH_sub(nh, "/camera/depth/image_raw", 1);
	
	// synchronization policy
	typedef sync_policies::ApproximateTime<Image,Image> MySyncPolicy;
	
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), RGB_sub, DEPTH_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
	// data publish for further processing 
	//pubData = nh.advertise<std_msgs::Float32MultiArray>("pubData", 10);
	// data publish for visualization
	marker_pub = nh.advertise<visualization_msgs::Marker>("ball_trajectory", 1);

	ros::spin();
	return 0;
}
// ################			main function END		################################################ 
