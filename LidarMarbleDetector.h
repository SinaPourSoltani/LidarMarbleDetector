//
// Created by Sina P. Soltani on 19/10/2019.
//

#ifndef PLAYGROUND_LIDARMARBLEDETECTOR_H
#define PLAYGROUND_LIDARMARBLEDETECTOR_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#define MAX_LIDAR_RANGE 10
#define SLOPE_DIFF 0.75
#define MIN_RADIUS 42
#define MAX_RADIUS 58
#define THRESHOLD 60
#define ROTATION_OFFSET 1
#define FOV (2.27 - (-2.27))
#define NUM_DATAPTS 200
#define ANGULAR_PREC FOV / NUM_DATAPTS
#define WINDOW_NAME "Lidar Plot"
#define IMAGE_COLS 2000
#define IMAGE_ROWS 2000
#define NO_TARGET 3

using namespace std;
using namespace cv;

class LidarMarbleDetector {
public:
    LidarMarbleDetector();
    LidarMarbleDetector(double * data, int size, int width, int height);

    void drawCircle(Point center, double r, bool isTarget);

    bool isInRange(double range);
    double determinant(Point a, Point b);
    double solve2LinEq(double a1, double b1, double a2, double b2);
    double calculateDistance(double x1, double y1, double x2, double y2);
    double getBiggestDiff(double * arr, int size);

    void perpendicularBisector(Point a, Point b, double *slopePtr, double *interceptPtr);
    auto calculateCenterAndRadiusOfCircle(Point a, Point b, Point c);

    int getIndexOfTargetCircle(Point * centers, int size);
    double getDirectionOfTargetCircle(Point circle);
    bool checkForCircles(int numPts, Point* points);
    double checkSegments();

    void getLidarSegments();
    void plotLidarData();
    void plotSegmentSlopes();

    double onSetData();
    double setLidarData(double * data);

    ~LidarMarbleDetector();

//protected:
    double * _lidarData;
    int _size;
    
    Mat _image;
    int _imageWidth;
    int _imageHeight;
    
    int _numSegments;
    int * _numPtsInSegment;
    Point ** _segments;
};


#endif //PLAYGROUND_LIDARMARBLEDETECTOR_H
