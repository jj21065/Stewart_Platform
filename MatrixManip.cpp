#include "MatrixManip.h"


#define MultiMatrix(x) tempx = T[0] * tri[i].x[0] + T[1] * tri[i].x[1] + T[2] * tri[i].x[2] + T[3] * 1;\
	tempy = T[4] * tri[i].x[0] + T[5] * tri[i].x[1] + T[6] * tri[i].x[2] + T[7] * 1; \
	tempz = T[8] * tri[i].x[0] + T[9] * tri[i].x[1] + T[10] * tri[i].x[2] + T[11] * 1; \
	tri[i].x[0] = tempx; \
	tri[i].x[1] = tempy; \
	tri[i].x[2] = tempz;
#define MultiMatrixPoint(a,T) tempx = T[0] * a.x + T[1] * a.y + T[2] * a.z + T[3] * 1;\
	tempy = T[4] * a.x + T[5] * a.y + T[6] * a.z + T[7] * 1; \
	tempz = T[8] * a.x + T[9] * a.y + T[10] * a.z + T[11] * 1; \
	a.x = tempx; \
	a.y = tempy; \
	a.z = tempz;
void matmulti(float *m1, float *m2,int msize ,float* returnM)
{
	for (int i = 0; i < msize; i++)
		for (int j = 0; j < msize; j++)
			for (int k = 0; k < msize; k++)
				returnM[i*msize + j] += m1[i*msize+k] * m2[j+k*msize];
}
float clamp(float x, float a, float b)
{
	return x < a ? a : (x < b ? x : b);
}
float dot(const float *a, const float *b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
float norm(const float *a)
{
	return sqrtf(dot(a, a));
}
void vassign(float *a, float x, float y, float z)
{
	a[0] = x; a[1] = y; a[2] = z;
}
void vassign(float *a, const float *b)
{
	a[0] = b[0]; a[1] = b[1]; a[2] = b[2];
}
void cross(float *a, const float *b, const float *c)
{
	a[0] = b[1] * c[2] - c[1] * b[2];
	a[1] = -b[0] * c[2] + c[0] * b[2];
	a[2] = b[0] * c[1] - c[0] * b[1];
}
void normalize(float *a)
{
	float l = norm(a);
	a[0] /= l; a[1] /= l; a[2] /= l;
}



void vectorCross(float*a, float*b, float*c, float*n)
{
	float v1[3] = { (a[0] - b[0]), (a[1] - b[1]), (a[2] - b[2]) };
	float v2[3] = { (a[0] - c[0]), (a[1] - c[1]), (a[2] - c[2]) };
	n[0] = v1[1] * v2[2] - v1[2] * v2[1];
	n[1] = v1[2] * v2[0] - v1[0] * v2[2];
	n[2] = v1[0] * v2[1] - v1[1] * v2[0];
}


