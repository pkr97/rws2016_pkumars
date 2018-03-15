
#include <armadillo>
#include <iostream>
#include <opencv2/opencv.hpp>
#include </usr/include/armadillo>
#include "simple_kalman.h"

using namespace std;
using namespace cv;
using namespace arma;

double k   =  0.00006;
double m   =  0.100;		//	SI UNITS
double g   = -9.80665;
double dt;

double x = 0;
double y = 0;
double z = 0;

double v_x  = 0;
double v_y  = 0;
double v_z  = 0;

double old_x  = 999.999;
double old_y  = 999.999;
double old_z  = 999.999;

bool Kalman_flag = false;

long timePred = 9999999999999;

int countTimes = 0;

arma::mat A = arma::mat(6,6,arma::fill::eye);
arma::mat P = arma::mat(6,6,arma::fill::eye);
arma::mat Q = arma::mat(6,6,arma::fill::eye);
arma::mat B = arma::mat(6,6,arma::fill::eye);
arma::mat H = arma::mat(6,6,arma::fill::eye);
arma::mat R = arma::mat(6,6,arma::fill::eye);
arma::mat X = arma::mat(6,1,arma::fill::zeros);
arma::mat U = arma::mat(6,1,arma::fill::zeros); 


// calculation block
void Kalman_calc(double x, double v_x, double y, double v_y, double z, double v_z, double dt) 
{

	U 	<<	0 			<< 	arma::endr
		<<	0			<< 	arma::endr
		<<	0			<< 	arma::endr
		<<	0			<< 	arma::endr
		<<	0.5*g*dt*dt << 	arma::endr
		<<	g*dt 		<< 	arma::endr;
		
	A 	<<	1 	<<	(dt-k*dt*dt/(2*m)) <<   0   <<  0                 <<  0 <<  0                  << endr
		<<	0 	<< 	(1 - k*dt/m)       <<   0   <<  0                 <<  0 <<  0                  << endr
		<<  0   <<  0                  <<	1 	<<	(dt-k*dt*dt/(2*m))<<  0 <<  0                  << endr
		<<  0   <<  0                  <<	0 	<< 	(1 - k*dt/m)      <<  0 <<  0                  << endr
		<<  0   <<  0                  <<   0   <<  0                 <<  1 <<	(dt-k*dt*dt/(2*m)) << endr
		<<  0   <<  0                  <<   0   <<  0                 <<  0 << 	(1 - k*dt/m)       << endr;


	arma::mat Z;
	arma::mat Pred_X;
	arma::mat Pred_P;
	arma::mat K;
	arma::mat I = arma::mat(6,6,arma::fill::eye);
	
	// measurement coming from sensor
	Z 	<< x   << arma::endr
		<< v_x << arma::endr
		<< y   << arma::endr
		<< v_y << arma::endr
		<< z   << arma::endr
		<< v_z << arma::endr;
	
	// prediction part
	Pred_X = A * X + B * U;											// prediction of states
	Pred_P = A * P * trans(A) + Q;                        			// prediction of state covariance matrix
	// update part
	K 	   = Pred_P * trans(H) * pinv(H * Pred_P * trans(H) + R);	// Kalman Gain Calculation
	X      = Pred_X + K * (Z - H * Pred_X);                    		// State update 
	P 	   = (I - K * H) * Pred_P;                					// State covariance matrix update 

}

// reinitialization
void reInitKalman()
{  	
	dt = 0.04;
	
	A = arma::mat(6,6,arma::fill::eye);
	P = arma::mat(6,6,arma::fill::eye);
	Q = arma::mat(6,6,arma::fill::eye);	
	B = arma::mat(6,6,arma::fill::eye);	
	H = arma::mat(6,6,arma::fill::eye);		
	R = arma::mat(6,6,arma::fill::eye);			
	X = arma::mat(6,1,arma::fill::zeros);			
	U = arma::mat(6,1,arma::fill::zeros);	
			
	A 	<<	1 	<<	(dt-k*dt*dt/(2*m)) <<   0   <<  0                 <<  0 <<  0                  << endr
		<<	0 	<< 	(1 - k*dt/m)       <<   0   <<  0                 <<  0 <<  0                  << endr
		<<  0   <<  0                  <<	1 	<<	(dt-k*dt*dt/(2*m))<<  0 <<  0                  << endr
		<<  0   <<  0                  <<	0 	<< 	(1 - k*dt/m)      <<  0 <<  0                  << endr
		<<  0   <<  0                  <<   0   <<  0                 <<  1 <<	(dt-k*dt*dt/(2*m)) << endr
		<<  0   <<  0                  <<   0   <<  0                 <<  0 << 	(1 - k*dt/m)       << endr; 
		
	U 	<<	0 			<< 	arma::endr
		<<	0			<< 	arma::endr
		<<	0			<< 	arma::endr
		<<	0			<< 	arma::endr
		<<	0.5*g*dt*dt << 	arma::endr
		<<	g*dt 		<< 	arma::endr;
		
	X 	<<	7.5 		<< 	arma::endr
		<<	0.5			<< 	arma::endr
		<< -0.5			<< 	arma::endr
		<<	3.0			<< 	arma::endr
		<<	3.0 		<< 	arma::endr
		<<	3.0 		<< 	arma::endr;
		
	P = 1000 * P;
	Q = 0.1  * Q;
	R = 0.2  * R;
}

void resetKalman()
{
	reInitKalman();

	x 	= 0;
	y 	= 0;
	z 	= 0;
	
	v_x  = 0;
	v_y  = 0;
	v_z  = 0;
	
	old_x  = 999.999;
	old_y  = 999.999;
	old_z  = 999.999;
	
	Kalman_flag = false;
	
	timePred = 99999999999999;
}


void predict(double DT, cv::Mat RGB) 
{
	dt = DT;
	//  cout << dt << endl;
	countTimes++;
	int t=50;
	arma::mat Prediction_X= A * X  + B * U ;
	
}
