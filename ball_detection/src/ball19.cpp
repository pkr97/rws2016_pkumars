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
	#include <sstream>
	#include <string>
	#include <cmath>
	#include <math.h>  
	// pcl headers
	#include <pcl_ros/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/filters/voxel_grid.h>
	#include <boost/foreach.hpp>
	#include <pcl/io/pcd_io.h>
	// visualization header
	#include <visualization_msgs/Marker.h>
	#include <pcl/common/time.h> // for watch(timer)
	// Message publication
	#include "std_msgs/MultiArrayLayout.h"
	#include "std_msgs/MultiArrayDimension.h"
	#include "std_msgs/Float32MultiArray.h"
	#include <visualization_msgs/Marker.h>
	#include <visualization_msgs/MarkerArray.h>
	// transform library
	#include <tf/message_filter.h>
	#include <tf/transform_listener.h>
	#include <tf/transform_broadcaster.h>
	#include <geometry_msgs/PointStamped.h>
	#include <boost/bind.hpp>
	#include <boost/scoped_ptr.hpp>
	#include "ros/ros.h"
	#include <gtest/gtest.h>
	// matrix library
	#include </usr/include/armadillo>

	ros::Publisher pubData;
	
	// point cloud definition
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	
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

void callback(const ImageConstPtr& image_depth_source)
{
	pcl::StopWatch watch;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Solve all of perception here... (ROS Format converted to OpenCV format here)
	cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_source)->image;
	
	cloud->width = 640;
	cloud->height = 480;
	cloud->is_dense = true;
	cloud->resize(cloud->width * cloud->height);


	
	for (int i=0;i<cloud->width;i++)
	{
		for (int j=0;i<cloud->height;i++)
		{
		// 3D coordinates from point cloud using depth value.. in Kinect coordinate space
		cloud->points[i*j].z = image_depth.at<ushort>(Point2d(i,j)) / 1000.0f;
		double z3Ddepth = cloud->points[i*j].z;
		
		cloud->points[i].x = (i - cx_d) * z3Ddepth / fx_d;
		cloud->points[i].x = (j - cy_d) * z3Ddepth / fy_d;
		
		
		//x3Ddepth = (u - cx_d) * z3Ddepth / fx_d;
		//y3Ddepth = (v - cy_d) * z3Ddepth / fy_d;

		}
	}

}

// ################			main function START		################################################ 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	// subscribe
	ros::Subscriber sub = nh.subscribe ("/camera/depth/image_raw", 1, callback);


	ros::spin();
	return 0;
}
// ################			main function END		################################################ 
