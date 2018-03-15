/*Drawing_1.cpp*/

/**
 * @file Drawing_1.cpp
 * @brief Simple geometric drawing
 * @author OpenCV team
 */
 
 #include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

using namespace cv;
using namespace std;

#define w 480
#define h 640

using namespace cv;

/// Function headers
void MyFilledCircle( Mat img, Point center );

/**
 * @function main
 * @brief Main function
 */
int main( void ){

  //![create_images]
  /// Windows names
  char atom_window[] = "Drawing 1: Atom";

  /// Create black empty images
  Mat atom_image = Mat::zeros( w, h, CV_8UC3 );

  /// 1.b. Creating circles
  MyFilledCircle( atom_image, Point( h/2, w/2) );
  //![draw_atom]


  /// 3. Display your stuff!
  imshow( atom_window, atom_image );
  moveWindow( atom_window, 0, 200 );
  
  
  Mat imgHSV;

   cvtColor(atom_image , imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  
  Mat imgThresholded;

   inRange(imgHSV, Scalar(0, 50, 50), Scalar(179, 255, 255), imgThresholded); //Threshold the image
  	
	vector<Point> locations;   // output, locations of non-zero pixels
	cv::findNonZero(imgThresholded, locations);
	cout << "white pixel indexes starts here" <<locations<<"white pixel indexes end here"<< endl;
	cout << "size of location matrix is = " <<locations.size()<<" ,"<< locations.size()<<endl;

  waitKey( 0 );
  return(0);
}


void MyFilledCircle( Mat img, Point center )
{
 int thickness = -1;
 int lineType = 8;

 circle( img,
         center,
         5,
         Scalar( 0, 0, 255 ),
         thickness,
         lineType ); // radius of circle w/10.0 changed to 10 pixels
}
