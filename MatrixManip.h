#pragma once
#include <math.h>

void matmulti(float *m1, float *m2, int msize, float* returnM);
float clamp(float x, float a, float b);
float dot(const float *a, const float *b);    //
float norm(const float *a);                      
void vassign(float *a, float x, float y, float z);
void vassign(float *a, const float *b);
void cross(float *a, const float *b, const float *c); //
void normalize(float *a);        // 

void vectorCross(float*a, float*b, float*c, float*n);

