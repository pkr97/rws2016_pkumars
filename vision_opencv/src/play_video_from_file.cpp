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

int main(int argc, char* argv[])
{
 //open the video file for reading
 VideoCapture cap("D:/My OpenCV Website/A Herd of Deer Running.mp4");
 
 // if not success, exit program
 if (cap.isOpened() == false)  
 {
  cout << "Cannot open the video file" << endl;
  cin.get(); //wait for any key press
  return -1;
 }
while(1){
 
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
  
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
 
    // Display the resulting frame
    imshow( "Frame", frame );
 
    // Press  ESC on keyboard to exit
    char c=(char)waitKey(25);
    if(c==27)
      break;
  }
  
  // When everything done, release the video capture object
  cap.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}
