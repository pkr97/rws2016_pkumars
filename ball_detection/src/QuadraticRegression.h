#ifndef PROJETO_OPENCV_QUADRATICREGRESSION_H
#define PROJETO_OPENCV_QUADRATICREGRESSION_H

#include <opencv2/opencv.hpp>
#include <list>

using namespace std;
using namespace cv;

class QuadraticRegression {
private:
    list<Point2d> points;

    double getSum(int, int);
public:
    QuadraticRegression();

    void addPoints(double, double);
    int getNumberOfPoints();
    void removeOldestPoint();
    void clearListOfPoints();
    double getValueA();
    double getValueB();
    double getValueC();

    virtual ~QuadraticRegression();
};

#endif //PROJETO_OPENCV_QUADRATICREGRESSION_H
