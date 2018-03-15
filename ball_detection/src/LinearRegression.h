#ifndef LINEAR_REGRESSION_H
#define LINEAR_REGRESSION_H

#include <opencv2/opencv.hpp>
#include <list>

using namespace std;
using namespace cv;

class LinearRegression
{
private:
    list<Point2d> points;

    double getSum(int, int);
public:
    LinearRegression();

    void addPoints(double, double);
    int getNumberOfPoints();
    void removeOldestPoint();
    void clearListOfPoints();
    double getValueA();
    double getValueB();
    
    virtual ~LinearRegression();
};

#endif //LINEAR_REGRESSION_H
