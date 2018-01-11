// created by Jacob in 2016/11/5
#include "Point.h"
#include <math.h>
using namespace myPoint;
Point::Point()
{
	x = 0;
	y = 0;
}
Point::Point(float ix, float iy,float iz)
{
	x = ix;
	y = iy;
	z = iz;
}
Point::~Point()
{}

float Point::operator[](int i)const
{
	if (i == 0)
		return x;
	if (i == 1)
		return y;
	if (i == 2)
		return z;
	
}


