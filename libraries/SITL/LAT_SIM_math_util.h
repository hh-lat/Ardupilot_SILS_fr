#ifndef MATH_UTIL_H
#define MATH_UTIL_H
#include<stdlib.h>
#include "math.h"
#include <stdint.h>
void array_initd(float * arr,int num);

extern float constrain_float1(float , float , float);

//void angular_vel_tf(float*,float*,float*,float*,float*,float*,float*,float*,float*,float*);
float sec(float);
int sign(float);

extern void MatrixMultiply(unsigned int usNoOfRowsOfA,unsigned int usNoOfColumnsOfA,float dpA[][usNoOfColumnsOfA],unsigned int usNoOfRowsOfB,unsigned int usNoOfColumnsOfB,float dpB[][usNoOfColumnsOfB],float dpC[usNoOfRowsOfA][usNoOfColumnsOfB]);
extern void MatrixMultiply_old(float *,unsigned int ,unsigned int ,float *,unsigned int ,unsigned int ,float *);

extern void transposed3x3(float[][3]);
extern void transposedmxnAToB(int m, int n, float[][n],float[][m]);
extern void transposedlxmxnAToB(int l, int m, int n, float[l][m][n], float[n][m][l]);
//extern void matinv(int, float**);
//void matinv_(int*,int*,int*,int*,float **);

extern void MatrixInverse3x3(float [][3],float [][3]);
extern void MatrixInverse4x4(float [4][4],float [4][4]);
extern void MatrixInverse6x6(float [][6],float [][6]);
//extern void MatrixInverse3x3(float[3],float[3]);
//extern void MatrixInverse4x4(float[4],float[4]);
//extern void MatrixInverse6x6(float[6],float[6]);
extern void cross_product(float [], float [], float []);
extern void dot_product(float [], float [], float *);

#endif
