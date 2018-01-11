// created by Jacob in 2016/11/5
#pragma once
#include <math.h>

namespace myPoint
{


	class Point
	{
	public: //
		float x, y, z;
	public:
		Point();//
		Point(float, float, float);//
		~Point();//
		//void Set_data(float, float);//
		float getDistance(Point p2)
		{
			return sqrt(pow(x - p2.x, 2) + pow(y - p2.y, 2) + pow(z - p2.z, 2));
		}
		float operator[](int i)const;//
	
		bool operator<(const Point& p2)
		{
			if (x < p2.x)
				return true;
			else if (x == p2.x && y < p2.y)
				return true;
			else if (x == p2.x && y == p2.y &&z < p2.z)
				return true;
			else
				return false;
		}

		bool operator==(const Point& p2)
		{
			if (x == p2.x && y == p2.y && z == p2.z)
				return true;
			else
				return false;
		}
	};
	
}

