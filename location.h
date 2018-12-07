
#ifndef GMIDETECTOR_LOADXML_H
#define GMIDETECTOR_LOADXML_H
#include "common.h"
void getLocations(Size sz, std::vector<std::vector<Point> > &locations, std::vector<Point>pts,
	double scale, Size winStride, Size padding, Size winSize, int levels);
#endif
