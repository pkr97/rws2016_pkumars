#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
 // Read the image file
 Mat image = imread("/home/pkumars/Pictures/lena.png");

 // Check for failure
 if (image.empty()) 
 {
  cout << "Could not open or find the image" << endl;
  cin.get(); //wait for any key press
  return -1;
 }

 String windowName = "The Guitar"; //Name of the window

 namedWindow(windowName); // Create a window

 imshow(windowName, image); // Show our image inside the created window.

 waitKey(0); // Wait for any keystroke in the window

 destroyWindow(windowName); //destroy the created window

 return 0;
}
