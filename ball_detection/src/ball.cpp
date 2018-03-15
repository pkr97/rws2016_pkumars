	#include <ros/ros.h>
	#include <image_transport/image_transport.h>
	#include <cv_bridge/cv_bridge.h>
	#include <sensor_msgs/image_encodings.h>
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>
	
	#include <sstream>
	#include <string>
	#include <iostream>
	
	//names that will appear at the top of each window
	static const std::string windowName = "Original Image";
	static const std::string windowName1 = "HSV Image";
	static const std::string windowName2 = "Thresholded Image";
	static const std::string windowName3 = "After Morphological Operations";
	static const std::string trackbarWindowName = "Trackbars";
	static const std::string OPENCV_WINDOW = "Raw Image window";
	static const std::string OPENCV_WINDOW_1 = "Edge Detected";
	static const std::string OPENCV_WINDOW_2 = "HSV Image";
	
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
	
	
	class Ball_Detector
	{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	
	public:
	Ball_Detector()
	: it_(nh_)
	{
	// Subscribe to input video feed and publish output video feed
	image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Ball_Detector::imageCb, this);
	
	//image_pub_ = it_.advertise("/edge_detector/raw_image", 1);
	
	//cv::namedWindow(OPENCV_WINDOW);
	
	}
	
	~Ball_Detector()
	{
	cv::destroyWindow(OPENCV_WINDOW);
	}
	
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
	
	cv_bridge::CvImagePtr cv_ptr;
	namespace enc = sensor_msgs::image_encodings;
	
	try
	{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}
	
	if (cv_ptr->image.rows > 400 && cv_ptr->image.cols > 600)
	{
	
	detect_edges(cv_ptr->image);
	image_pub_.publish(cv_ptr->toImageMsg());
	
	}
	}
	
	
void createTrackbars()
{
	//create window for trackbars


    namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX);
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX);
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX);
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX);
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX);
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX);


}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{

	cv::Mat temp;
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
				// pixel coordinates of center of ball
				std::cout<<x<<" "<<y<<std::endl;

			}
			//let user know you found an object
			if(objectFound ==true){
				putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				//draw object location on screen
				drawObject(x,y,cameraFeed);}

		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

string intToString(int number){


	std::stringstream ss;
	ss << number;
	return ss.str();
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
	
	
	void detect_edges(cv::Mat img)
	{
	createTrackbars();
	cv::Mat src, src_gray;
	cv::Mat srcl, src_clr;
	cv::Mat dst, detected_edges;
	
	//some boolean variables for different functionality within this
	//program
    bool trackObjects = false;
    bool useMorphOps = false;
    
	cv::Mat cameraFeed;
	//matrix storage for HSV image
	cv::Mat HSV;
	//matrix storage for binary threshold image
	cv::Mat threshold;
	
	
	//x and y values for the location of the object
	int x=0, y=0;
	//create slider bars for HSV filtering
	createTrackbars();
	
	
	img.copyTo(cameraFeed);
	//convert frame from BGR to HSV colorspace
	cv::cvtColor(cameraFeed,HSV,CV_BGR2HSV);
	//filter HSV image between values and store filtered image to
	//threshold matrix
	cv::inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
	
	cv::Mat erodeElement1 = getStructuringElement( MORPH_RECT,Size(3,3));
	cv::Mat erodeElement2 = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement1 = getStructuringElement( MORPH_RECT,Size(8,8));
	cv::Mat dilateElement2 = getStructuringElement( MORPH_RECT,Size(8,8));
	
	cv::erode(threshold,threshold,erodeElement1);
	cv::erode(threshold,threshold,erodeElement2);
	
	
	cv::dilate(threshold,threshold,dilateElement1);
	cv::dilate(threshold,threshold,dilateElement2);
	
	cv::imshow(windowName2,threshold);
	
	trackFilteredObject(x,y,threshold,cameraFeed);
	
	////show frames 
	cv::imshow(windowName,cameraFeed);
		
	cv::waitKey(3);
	}	
	
	};
	
	
	int main(int argc, char** argv)
	{
	ros::init(argc, argv, "Ball_Detector");
	Ball_Detector ic;
	ros::spin();
	return 0;
	}
	
