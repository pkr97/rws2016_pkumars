#include "QuadraticRegression.h"

QuadraticRegression::QuadraticRegression() {}

void QuadraticRegression::addPoints(double x, double y) {
    this->points.push_back(Point2d(x, y));
}

int QuadraticRegression::getNumberOfPoints() {
    return this->points.size();
}


void QuadraticRegression::clearListOfPoints() {
    this->points.clear();
}


void QuadraticRegression::removeOldestPoint() {
    this->points.pop_front();
}

double QuadraticRegression::getValueA() {
    if(this->points.size() < 3)
        throw -1;

    double s40 = this->getSum(4, 0);
    double s30 = this->getSum(3, 0);
    double s20 = this->getSum(2, 0);
    double s10 = this->getSum(1, 0);
    double s00 = this->points.size();

    double s21 = this->getSum(2, 1);
    double s11 = this->getSum(1, 1);
    double s01 = this->getSum(0, 1);

    return (s21*(s20 * s00 - s10 * s10) -
            s11*(s30 * s00 - s10 * s20) +
            s01*(s30 * s10 - s20 * s20))
            /
           (s40*(s20 * s00 - s10 * s10) -
            s30*(s30 * s00 - s10 * s20) +
            s20*(s30 * s10 - s20 * s20));
}

double QuadraticRegression::getValueB() {
    if (this->points.size() < 3)
        throw -1;

    double s40 = this->getSum(4, 0);
    double s30 = this->getSum(3, 0);
    double s20 = this->getSum(2, 0);
    double s10 = this->getSum(1, 0);
    double s00 = this->points.size();

    double s21 = this->getSum(2, 1);
    double s11 = this->getSum(1, 1);
    double s01 = this->getSum(0, 1);

    return (s40*(s11 * s00 - s01 * s10) -
            s30*(s21 * s00 - s01 * s20) +
            s20*(s21 * s10 - s11 * s20))
           /
           (s40 * (s20 * s00 - s10 * s10) -
            s30 * (s30 * s00 - s10 * s20) +
            s20 * (s30 * s10 - s20 * s20));
}

double QuadraticRegression::getValueC() {
    if (this->points.size() < 3)
        throw -1;

    double s40 = this->getSum(4, 0);
    double s30 = this->getSum(3, 0);
    double s20 = this->getSum(2, 0);
    double s10 = this->getSum(1, 0);
    double s00 = this->points.size();

    double s21 = this->getSum(2, 1);
    double s11 = this->getSum(1, 1);
    double s01 = this->getSum(0, 1);

    return (s40 * (s20 * s01 - s10 * s11) -
            s30 * (s30 * s01 - s10 * s21) +
            s20 * (s30 * s11 - s20 * s21))
           /
           (s40 * (s20 * s00 - s10 * s10) -
            s30 * (s30 * s00 - s10 * s20) +
            s20 * (s30 * s10 - s20 * s20));
}

double QuadraticRegression::getSum(int x, int y) {
    double Sum = 0;
    for(Point2f point : this->points) {
        Sum += pow(point.x, x) * pow(point.y, y);
    }

    return Sum;
}

QuadraticRegression::~QuadraticRegression() {}
