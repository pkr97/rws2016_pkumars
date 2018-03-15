#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <string>
#include <iostream>

// ROS synchronization headers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

	// array manipulation
	#include "std_msgs/Float32MultiArray.h"
	
	#include <tf/transform_broadcaster.h>


// standard namespace for c++, ROS messages transport, ROS message filters, OpenCV 
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

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

// publisher defined
ros::Publisher ballCord;

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed

}

string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
}
void createTrackbars(){
	//create window for trackbars


    namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN");
	sprintf( TrackbarName, "H_MAX");
	sprintf( TrackbarName, "S_MIN");
	sprintf( TrackbarName, "S_MAX");
	sprintf( TrackbarName, "V_MIN");
	sprintf( TrackbarName, "V_MAX");
	
	//sprintf( TrackbarName, "H_MIN", H_MIN);
	//sprintf( TrackbarName, "H_MAX", H_MAX);
	//sprintf( TrackbarName, "S_MIN", S_MIN);
	//sprintf( TrackbarName, "S_MAX", S_MAX);
	//sprintf( TrackbarName, "V_MIN", V_MIN);
	//sprintf( TrackbarName, "V_MAX", V_MAX);
	
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


}
void drawObject(int x, int y,Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

	putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}
void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	


}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed, Mat image_depth){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
					refArea = area;
				}else objectFound = false;


			}
			//let user know you found an object
			if(objectFound ==true)
			{
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,cameraFeed);
			
				geometry_msgs::PointStamped point_out;
				
				// Coordinates defined
				double x3Ddepth, y3Ddepth, z3Ddepth, x3DRGB, y3DRGB, z3DRGB;
				
				if (x>20 && y<460 && x>20 && y<620 && objectFound)
				{
					// 3D coordinates from point cloud using depth value.. in Kinect coordinate space
					z3Ddepth = image_depth.at<ushort>(Point2d(x,y)) / 1000.0f;
					x3Ddepth = (x - cx_d) * z3Ddepth / fx_d;
					y3Ddepth = (y - cy_d) * z3Ddepth / fy_d;
			
					// 3D coordinates from depth frame to RGB frame.. (Transformed using Rotation and Translation matrix)
					x3DRGB = (x3Ddepth*R[0][0])+(y3Ddepth*R[0][1])+(z3Ddepth*R[0][2])+T[0][0];
					y3DRGB = (x3Ddepth*R[1][0])+(y3Ddepth*R[1][1])+(z3Ddepth*R[1][2])+T[1][0];
					z3DRGB = (x3Ddepth*R[2][0])+(y3Ddepth*R[2][1])+(z3Ddepth*R[2][2])+T[2][0];
					
					if (z3DRGB !=0)
					{	
						// coordinates published at time instant
						
						// KinectEstimated (KE) header stamp and frame id
						point_out.header.stamp = ros::Time::now();
						point_out.header.frame_id = "/ballCord";
						
						point_out.point.x = x3DRGB;
				        point_out.point.y = y3DRGB;
				        point_out.point.z = z3DRGB;
					
						ballCord.publish(point_out);
				
						printf("\n%3d - Flying ball - %6f %6f %6f",x3DRGB, y3DRGB, z3DRGB);
					}
					else
					{
						// coordinates published at time instant
						
						// KinectEstimated (KE) header stamp and frame id
						point_out.header.stamp = ros::Time::now();
						point_out.header.frame_id = "/ballCord";
						
						point_out.point.x = 0.0;
				        point_out.point.y = 0.0;
				        point_out.point.z = 0.0;
					
						ballCord.publish(point_out);
					}
				}
			
			
			}
				
			

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

void callback(const ImageConstPtr& image_rgb, const ImageConstPtr& image_depth_source)
{

	//some boolean variables for different functionality within this
	//program
    bool trackObjects = false;
    bool useMorphOps = false;
	//matrix storage for HSV image
	Mat HSV;
	//matrix storage for binary threshold image
	Mat threshold;
	//x and y values for the location of the object
	int x=0, y=0;
	//create slider bars for HSV filtering
	createTrackbars();
	
	// Solve all of perception here... (ROS Format converted to OpenCV format here)
	cv::Mat image_color = cv_bridge::toCvCopy(image_rgb)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_source)->image;
	
	while(1){
		//convert frame from BGR to HSV colorspace
		cvtColor(image_color,HSV,COLOR_BGR2HSV);
		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if(useMorphOps)
		morphOps(threshold);
		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if(trackObjects)
			trackFilteredObject(x,y,threshold,image_color,image_depth);

		//show frames 
		imshow(windowName2,threshold);
		imshow(windowName,image_color);
		imshow(windowName1,HSV);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		waitKey(30);
	}
	
	
}
	
// ################			main function START		################################################ 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "hit_detection");

	ros::NodeHandle nh;
	
	// topic subscription
	message_filters::Subscriber<Image> RGB_sub(nh, "/camera/rgb/image_color", 1);
	message_filters::Subscriber<Image> DEPTH_sub(nh, "/camera/depth/image_raw", 1);
	
	// synchronization policy
	typedef sync_policies::ApproximateTime<Image,Image> MySyncPolicy;
	
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), RGB_sub, DEPTH_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	
	// data publish for further processing 
	ballCord = nh.advertise<geometry_msgs::PointStamped>("/ballCord", 1);	

	ros::spin();
	return 0;
}
// ################			main function END		################################################ 
	
