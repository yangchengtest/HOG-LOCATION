//
// Created by magicyang on 2018/11/16.
//
#ifndef GMIDETECTOR_COMMON_H
#define GMIDETECTOR_COMMON_H

#include <opencv2/core/core.hpp>
using namespace cv;
using namespace std;

typedef struct tagRect
{
	vector<Point> pts;
    int regionType;
    int dectorDense;
    double scale;
	double resize;
	Size padding;
	Size stride;
}Region_T;
#endif 
