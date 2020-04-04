//
// Created by lihao on 19-7-9.
//

#ifndef OCCMAPTRANSFORM_H
#define OCCMAPTRANSFORM_H

#include <iostream>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


//-------------------------------- Class---------------------------------//
class OccupancyGridParam{

public: // Interface
    void GetOccupancyGridParam(nav_msgs::OccupancyGrid OccGrid);
    void Image2MapTransform(Point& src_point, Point2d& dst_point);
    void Map2ImageTransform(Point2d& src_point, Point& dst_point);

private: // Private function

public:  // Public variable
    double resolution;
    int height;
    int width;
    // Origin pose
    double x;
    double y;
    double theta;

private: // Private variable
    // Transform
    Mat R;
    Mat t;

};



#endif //OCCMAPTRANSFORM_H
