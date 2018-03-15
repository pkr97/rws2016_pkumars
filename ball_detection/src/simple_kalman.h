#ifndef SIMPLE_KALMAN_H_
#define SIMPLE_KALMAN_H_

#include <armadillo>
#include <opencv2/opencv.hpp>
#include <iostream>
#include </usr/include/armadillo>

using namespace std;
using namespace cv;
using namespace arma;

extern int countTimes;

extern double k;
extern double m; 
extern double g;
extern double dt;

extern double x;
extern double y;
extern double z;

extern double v_x;
extern double v_y;
extern double v_z;

extern double old_x;
extern double old_y;
extern double old_z;

extern bool Kalman_flag;

extern arma::mat A;
extern arma::mat P;
extern arma::mat Q;
extern arma::mat B;
extern arma::mat H;
extern arma::mat R;
extern arma::mat X;
extern arma::mat U;

void Kalman_calc(double x, double v_x, double y, double v_y, double z, double v_z, double dt);
void resetKalman();
void reInitKalman();
void predict(double DT, cv::Mat RGB);

#endif
