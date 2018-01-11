#include <math.h>
#include "Point.h"
#include "myServo.h"
#include "MatrixManip.h"

//using namespace myPoint;
class Stewart
{
public:
	myPoint::Point BaseOrigin;
	myPoint::Point PlateOrigin;
	myPoint::Point BasePoint[6];
	myPoint::Point PlatePoint[6];
	myServo servo[6];


	myPoint::Point targetPoint;
	float targetPitch;
	float targetRoll;
	float targetYaw;
public:
	Stewart()
	{
		init_stewart();
	}
	void init_stewart()
	{
		init_servo();
		targetPoint.x = 0;
		targetPoint.y = 0;
		targetPoint.z = 70;

		targetPitch = 0;
		targetRoll = 0;
		targetYaw = 0;


		BaseOrigin = myPoint::Point(0, 0, 0);
		PlateOrigin = myPoint::Point(0, 0, 0);

		BasePoint[0] = myPoint::Point(45.89, 67.02, 0);
		BasePoint[1] = myPoint::Point(80.99, 6.23, 0);
		BasePoint[2] = myPoint::Point(35.1, -73.25, 0);
		BasePoint[3] = myPoint::Point(-35.1, -73.25, 0);
		BasePoint[4] = myPoint::Point(-80.99, 6.23, 0);
		BasePoint[5] = myPoint::Point(-45.89, 67.02, 0);

		PlatePoint[0] = myPoint::Point(15, 59.98, 0);
		PlatePoint[1] = myPoint::Point(59.44, -17, 0);
		PlatePoint[2] = myPoint::Point(44.44, -42.98, 0);
		PlatePoint[3] = myPoint::Point(-44.44, -42.98, 0);
		PlatePoint[4] = myPoint::Point(-59.44, -17, 0);
		PlatePoint[5] = myPoint::Point(-15, 59.98, 0);

	}
	void init_servo()
	{
		servo[0].thida_S = 120 * 3.14 / 180;
		servo[1].thida_S = -60 * 3.14 / 180;
		servo[2].thida_S = 0 * 3.14 / 180;
		servo[3].thida_S = 180*3.14/180;
		servo[4].thida_S = -120 * 3.14 / 180;
		servo[5].thida_S = 60 * 3.14 / 180;
	}
	void initStruct(myPoint::Point* baseP,myPoint::Point* plateP)
	{
		for (int i = 0; i < 6; i++)
		{
			BasePoint[i] = baseP[i];
			PlatePoint[i] = plateP[i];
		}
	}
	void getTransMat(float* R,float *T)
	{
		float OriginOffset[3] = {targetPoint.x,targetPoint.y,targetPoint.z};
		float R_yaw[9] = { cos(targetYaw), -sin(targetYaw), 0, sin(targetYaw), cos(targetYaw), 0, 0, 0, 1 };
		float R_roll[9] = { cos(targetRoll), 0, sin(targetRoll), 0, 1, 0, -sin(targetRoll), 0, cos(targetRoll)};
		float R_pitch[9] = { 1, 0, 0, 0, cos(targetPitch),-sin(targetPitch),0,sin(targetPitch),cos(targetPitch)};
		
		float rr[9] = { 0 };
		matmulti(R_yaw, R_roll, 3, rr);
		matmulti(rr, R_pitch, 3, R);
		for (int i = 0; i < 3; i++)
			T[i] = OriginOffset[i];
	}
};
