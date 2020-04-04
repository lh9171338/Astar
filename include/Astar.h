//
// Created by lihao on 19-7-9.
//

#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


namespace pathplanning{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

struct Node{
    Point point;  // node coordinate
    int F, G, H;  // cost
    Node* parent; // parent node

    Node(Point _point = Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

struct AstarConfig{
    bool Euclidean;         // true/false
    int OccupyThresh;       // 0~255
    int InflateRadius;      // integer

    AstarConfig(bool _Euclidean = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Euclidean(_Euclidean), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }
};

class Astar{

public:
    // Interface function
    void InitAstar(Mat& _Map, AstarConfig _config = AstarConfig());
    void InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config = AstarConfig());
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path);
    void DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask = noArray(), Scalar color = Scalar(0, 0, 255),
            int thickness = 1, Scalar maskcolor = Scalar(255, 255, 255));

private:
    void MapProcess(Mat& Mask);
    Node* FindPath();
    void GetPath(Node* TailNode, vector<Point>& path);

private:
    //Object
    Mat Map;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    AstarConfig config;
    vector<Node*> OpenList;  // open list
    vector<Node*> PathList;  // path list
};

}




#endif //ASTAR_H
