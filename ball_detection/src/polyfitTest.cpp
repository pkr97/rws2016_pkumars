#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
// c++ headers
#include <iostream>
// ROS headers
#include <ros/ros.h>

#include "precomp.hpp"
using namespace cv;
using namespace std;

int paddedHeight = 1 ; 
int paddedWidth = 5 ;  
int n = paddedWidth;

void cvPolyfit(cv::Mat &src_x, cv::Mat &src_y, cv::Mat &dst, int order)
{
	CV_FUNCNAME("cvPolyfit");
	__CV_BEGIN__;
	{
		CV_ASSERT((src_x.rows>0)&&(src_y.rows>0)&&(src_x.cols==1)&&(src_y.cols==1)
				&&(dst.cols==1)&&(dst.rows==(order+1))&&(order>=1));
		Mat X;
		X = Mat::zeros(src_x.rows, order+1,CV_32FC1);
		Mat copy;
		for(int i = 0; i <=order;i++)
		{
			copy = src_x.clone();
			pow(copy,i,copy);
			Mat M1 = X.col(i);
			copy.col(0).copyTo(M1);
		}
		Mat X_t, X_inv;
		transpose(X,X_t);
		Mat temp = X_t*X;
		Mat temp2;
		invert (temp,temp2);
		Mat temp3 = temp2*X_t;
		Mat W = temp3*src_y;
#ifdef DEBUG
		cout<<"PRINTING INPUT AND OUTPUT FOR VALIDATION AGAINST MATLAB RESULTS\n";
		cout<<"SRC_X: "<<src_x<<endl;
		cout<<"SRC_Y: "<<src_y<<endl;
		cout<<"X: "<<X<<endl;
		cout<<"X_T: "<<X_t<<endl;
		cout<<"W:"<<W<<endl;
#endif
		dst = W.clone();
	}
	__CV_END__;
}


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "vision_node");
	
	ros::NodeHandle nh;	
		
	Mat time = (Mat_<float>(10,1, CV_64F) <<  0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		
	cout<<"Time.rows: "<<time.rows<<"\nTime.cols: "<<time.cols<<endl;
	
	Mat prices = (Mat_<float>(10,1, CV_64F) <<  0.0, 1.0, 8.0, 27.0, 64.0, 125.0, 216.0, 343.0, 512.0, 729.0);
	
	cout<<"Prices.rows: "<<prices.rows<<"\nPrices.cols: "<<prices.cols<<endl;

	int fit_order = 3;
	Mat fit_weights(fit_order+1,1,CV_32FC1);
	cvPolyfit(time,prices,fit_weights,fit_order);
	cout<<"Weights of fit of order: "<<fit_weights<<endl;
	
	//cv::Mat f = cv::Mat(paddedHeight,paddedWidth,CV_64FC2);
	
	
	//float yValue = 0;
	
	//for(int j=0;j<n;j++)
	//{
		//f.at<cv::Vec2d>(0,j) = yValue;
		//cout<<"Matrix f is "<<f<<endl;
		//cout<<"f.rows: "<<f.rows<<"\n f.cols: "<<f.cols<<endl;
		//++yValue;
	//}
	
	
	
	return 0;
}
