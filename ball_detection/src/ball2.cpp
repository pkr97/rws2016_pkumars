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
	#include <pcl/common/time.h> // for watch(timer)
	#include <pcl/console/print.h>
	#include <pcl_ros/point_cloud.h>
	#include <pcl/point_types.h>
	// Message publication
	#include "std_msgs/Float32MultiArray.h"
	#include <geometry_msgs/PoseStamped.h>
	
	ros::Publisher pubData;
	ros::Publisher ball3D; // publish end-effector
	
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
// ########################## Ball Extraction parameters END		############################

// ###################	Image Processing and 3D position Extraction callback function		#####

void callback(const ImageConstPtr& image_rgb, const sensor_msgs::PointCloud2::ConstPtr& pCloud)
{
	pcl::StopWatch watch;

	// Solve all of perception here... (ROS Format converted to OpenCV format here)
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
	std_msgs::Float32MultiArray dataArray;
	// data clear 
	dataArray.data.clear();
	
	// ###################        Image processing starts here		############################
	
	// new matrix declaration for copy and processing 
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
	double x3Ddepth, y3Ddepth, z3Ddepth, x3DRGB, y3DRGB, z3DRGB;
	
	if (u>20 && u<460 && v>20 && v<620 && ballfound)
	{
	// raw depth value from RGB pixel coordinate
	int i = (u) + (v)*cloud->width;
	
	// 3D coordinates from point cloud using depth value
	x3Ddepth = (float)cloud->points[i].x;
	y3Ddepth = (float)cloud->points[i].y;
	z3Ddepth = (float)cloud->points[i].z;
	
	// 3D coordinates from depth frame to RGB frame
	x3DRGB = (x3Ddepth*R[0][0])+(y3Ddepth*R[0][1])+(z3Ddepth*R[0][2])+T[0][0];
	y3DRGB = (x3Ddepth*R[1][0])+(y3Ddepth*R[1][1])+(z3Ddepth*R[1][2])+T[1][0];
	z3DRGB = (x3Ddepth*R[2][0])+(y3Ddepth*R[2][1])+(z3Ddepth*R[2][2])+T[2][0];
	}
	// variable declaration
	double xRobot, yRobot, zRobot;
	// Kinect frame to Robot frame..(Transformation)
	//xRobot = z3DRGB;
	//yRobot = x3DRGB;
	//zRobot = y3DRGB;
	
	xRobot = x3DRGB;
	yRobot = y3DRGB;
	zRobot = z3DRGB;
	
	dt=dt+watch.getTimeSeconds(); // running time instant 
	
	// ############ 	Ball 3D position extraction process ends here 		########################
	
	// ############ 	Ball 3D position publish with conditions 		############################
	
	if(std::isnan(xRobot)) // if nan value comes from depth image 
	{
		ROS_INFO("\n ball not found in ROI \n");
		// data matrix set to zero
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(dt);
		pubData.publish(dataArray);
	}
	else if (u>20 && u<460 && v>20 && v<620 && ballfound && zRobot > 0) // valid and acepted condition
	{
		// coordinates published at time instant
		dataArray.data.push_back(xRobot);
		dataArray.data.push_back(yRobot);
		dataArray.data.push_back(zRobot);
		dataArray.data.push_back(dt);
		pubData.publish(dataArray);
		
		printf("\n - Flying ball - %6f %6f %6f %6f", xRobot, yRobot, zRobot, dt);
		//pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
	}
	else if (xRobot<0) // invalid values
	{	
		// coordinates published at time instant
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(dt);
		pubData.publish(dataArray);
		
		printf("\n Not Valid point \n");
	}
	else // something is wrong 
	{	
		// coordinates published at time instant
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(0.0d);
		dataArray.data.push_back(dt);
		pubData.publish(dataArray);
		
		printf("\n something is wrong \n");
	}
	// ############			Ball 3D position publish with conditions END		#####################
	
	// ############			Image Dispaly and Refresh starts here		#############################

	imshow("Color Image", image_color); //show the original image
	
	imshow("Threshold Image", threshold); //show the original image
	
	// Frequency display 
    pcl::console::print_highlight ("\n frequency: %f\n", 1/(watch.getTimeSeconds()));
    
    
    		
		// publisher definition
		geometry_msgs::PoseStamped msg1;
	
		// KinectEstimated (KE) header stamp and frame id
		msg1.header.stamp = ros::Time::now();
		msg1.header.frame_id = "/base2";
	
		// KE Position 
		msg1.pose.position.x = 0.1; 
		msg1.pose.position.y = 0.1; 
		msg1.pose.position.z = 0.1;  
		// KE Orientation 
		msg1.pose.orientation.x = 0.0; 
		msg1.pose.orientation.y = 0.0; 
		msg1.pose.orientation.z = 0.0;
		msg1.pose.orientation.w = 1.0;   

		// EE Publish 
		ball3D.publish(msg1);
	
    
    // for image refreshing 
    cv::waitKey(1);
    // ############			Image Dispaly and Refresh ends here		################################
}

// ################			main function START		################################################ 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	
	message_filters::Subscriber<Image> RGB_sub(nh, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud_sub(nh, "/camera/depth/points", 1);

	typedef sync_policies::ApproximateTime<Image, sensor_msgs::PointCloud2> MySyncPolicy;
	
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), RGB_sub, PointCloud_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	// data publish for further processing 
	pubData = nh.advertise<std_msgs::Float32MultiArray>("/ballCord", 1);
	ball3D = nh.advertise<geometry_msgs::PoseStamped>("/ball3D", 1);

	ros::spin();
	return 0;
}
// ################			main function END		################################################ 
